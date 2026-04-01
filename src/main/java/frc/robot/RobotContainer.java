// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Static imports for unit types (e.g. MetersPerSecond, RadiansPerSecond) used in drivetrain tuning.
import static edu.wpi.first.units.Units.*;

// CTRE Swerve: how each module applies throttle (open-loop voltage vs closed-loop) and request types.
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest; // Fluent API for swerve setpoints (field-centric, brake, idle, etc.).

// Choreo: factory builds auto routines; chooser exposes them on SmartDashboard; routine wraps a trajectory.
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;

// WPILib math: PID for heading hold; Rotation2d for field-centric seed and zero heading.
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance; // Root NT access for custom topics.
import edu.wpi.first.networktables.StructPublisher; // Publishes a struct-encoded object over NetworkTables.
import edu.wpi.first.wpilibj.DriverStation; // DS state: joystick connected, alliance, match phase, etc.
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Dashboard widgets (e.g. auto chooser, booleans).
import edu.wpi.first.wpilibj2.command.Command; // Composable robot action.
import edu.wpi.first.wpilibj2.command.CommandScheduler; // Schedules the command selected for auto.
import edu.wpi.first.wpilibj2.command.Commands; // Factory helpers (runOnce, sequence, etc.).
import edu.wpi.first.wpilibj2.command.RunCommand; // Repeats a Runnable every loop while requiring a subsystem.
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller; // PS5 controller as triggers for Command bindings.
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers; // Event triggers when robot enters disabled, etc.
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction; // Forward/reverse for CTRE SysId helpers.

import frc.robot.generated.TunerConstants; // Phoenix Tuner X generated swerve constants + module CAN IDs.
import frc.robot.lib.SwerveTelemetry; // Custom struct for Reflect / NT mirroring of swerve state.
import frc.robot.subsystems.CommandSwerveDrivetrain; // Command-based swerve subsystem wrapping CTRE drivetrain.
import frc.robot.subsystems.Shooter; // Shooter + indexer TalonFX subsystem.
import frc.robot.subsystems.Vision; // Vision processing (constructed with drivetrain for pose or aiming).

/**
 * Wires hardware subsystems to driver input and autonomous selection.
 * Default commands run whenever no other command is using the same subsystem.
 */
public class RobotContainer {

    

    // ── Speeds ────────────────────────────────────────────────────────────────
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // Max yaw rate: 0.75 rot/s converted to rad/s for SwerveRequest rotational component.
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // ── Heading hold PID (tune kP / kD as needed) ─────────────────────────────
    private final PIDController headingController = new PIDController(0.05, 0.0, 0.001);
    // Last heading to hold (radians), updated whenever the driver actively rotates.
    private double targetHeadingRad = 0.0;
    // Right-stick magnitude below this uses heading-hold instead of manual omega.
    private static final double ROTATION_DEADBAND = 0.1;

    // ── Subsystems ────────────────────────────────────────────────────────────
    private final Shooter motors = new Shooter();
    // private final climber climber = new climber();

    // Single swerve drivetrain instance from generated constants (modules + pigeon id inside TunerConstants).
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Vision subsystem; passed drivetrain so it can consume pose or send alignment corrections.
    private final Vision vision = new Vision(drivetrain);
    // Command built from AutoChooser when autonomous starts (scheduled from Robot.autonomousInit delegation).
    private Command _autonomousCommand;

    // ── Swerve requests ───────────────────────────────────────────────────────
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1) // Ignore small joystick translation noise (10% of max speed).
            .withRotationalDeadband(MaxAngularRate * 0.1) // Ignore small rotation stick noise.
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Voltage duty to drive motors (not closed-loop vel).
    // Holds modules in brake/coast per CTRE idle config — used as the disabled idle request.
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // Points all modules in a direction (reserved for future bindings).
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // No output — safe neutral request while disabled.
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    // ── Telemetry ─────────────────────────────────────────────────────────────
    public final Telemetry logger = new Telemetry(MaxSpeed);
    // Mutable snapshot published to NT for external tools (Reflect, custom dashboards).
    private final SwerveTelemetry m_reflectTelemetry = new SwerveTelemetry();
    // Publishes SwerveTelemetry struct under NetworkTables "Robot/SwerveTelemetry".
    private final StructPublisher<SwerveTelemetry> m_telemetryPublisher =
        NetworkTableInstance.getDefault()
            .getTable("Robot")
            .getStructTopic("SwerveTelemetry", SwerveTelemetry.struct)
            .publish();

    // ── Controller ────────────────────────────────────────────────────────────
    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    // ── Shooter direction state ───────────────────────────────────────────────
    private double indexDirection = 1.0;
    // +1 / -1 flips shooter wheel direction when the driver toggles.
    private double shooterDirection = 1.0;

    // ── Choreo auto ───────────────────────────────────────────────────────────
    private final AutoFactory autoFactory;
    // Dashboard dropdown that picks which AutoRoutine runs in autonomous.
    private final AutoChooser autoChooser;

    // ─────────────────────────────────────────────────────────────────────────
    public RobotContainer() {
        // Allow PID to wrap across ±π so shortest path is always used for heading error.
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        // Choreo needs the robot pose, reset, and a method to follow samples — mirror=true flips paths on red alliance.
        autoFactory = new AutoFactory(
            () -> drivetrain.getState().Pose,   // Current field pose from swerve odometry.
            drivetrain::resetPose,              // Snap pose (e.g. path start) when a routine requests it.
            drivetrain::followChoreoSample,     // Applies one trajectory sample to the modules each tick.
            true,                               // Mirror trajectories when on red alliance.
            drivetrain                          // Subsystem required while auto routines run.
        );

        autoChooser = new AutoChooser();
        // Register named routines; names appear on the dashboard and map to deploy/choreo/*.traj files.
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.addRoutine("leave", () -> MyPath(autoFactory));

        // Duplicate putData is harmless but redundant — keeps chooser visible if code path changes.
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Bind joystick buttons and set subsystem default commands.
        configureBindings();

        // Align field-centric "forward" with the robot's current heading at startup.
        drivetrain.seedFieldCentric(Rotation2d.kZero);
    }

    // ─────────────────────────────────────────────────────────────────────────
    private void configureBindings() {

        // ── Default drive command with heading-hold ───────────────────────────
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Negative right X: WPILib rotation positive is CCW; controller axis sign convention handled here.
                double rotInput = -joystick.getRightX() * MaxAngularRate;

                if (Math.abs(joystick.getRightX()) > ROTATION_DEADBAND) {
                    // Manual turn: remember heading so releasing the stick doesn't snap the robot backward.
                    targetHeadingRad = drivetrain.getState().Pose.getRotation().getRadians();
                    return drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Forward stick → +X in field frame.
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Left stick → +Y in field frame.
                        .withRotationalRate(rotInput);
                } else {
                    // Heading hold: PID computes omega to drive current heading toward targetHeadingRad.
                    double correction = headingController.calculate(
                        drivetrain.getState().Pose.getRotation().getRadians(),
                        targetHeadingRad
                    );
                    return drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(correction);
                }
            })
        );

        // While disabled, continuously send Idle so motors stay in configured neutral/brake mode.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true) // Still run this command even though robot is disabled.
        );

        // Whenever no other command needs the shooter, map triggers to wheel and indexer speeds.
        motors.setDefaultCommand(
            new RunCommand(() -> {
                if (!DriverStation.isJoystickConnected(0)) return; // Avoid null/zero behavior with no DS joystick.

                // PS5 triggers often span [-1,1]; +1 shifts usable range for indexing as a positive magnitude.
                double leftTrigger  = joystick.getL2Axis() + 1.0;
                double rightTrigger = joystick.getR2Axis() + 1.0;

                leftTrigger  *= indexDirection;  // Flip indexer direction if driver toggled.
                rightTrigger *= shooterDirection; // Flip shooter direction if driver toggled.

                SmartDashboard.putBoolean("indexer", indexDirection == 1.0);
                SmartDashboard.putBoolean("shooter", shooterDirection == 1.0);

                final double MAX_SHOOTER_RPS = 80.0; // Flywheel velocity target scale (tune for your mechanism).
                motors.setLeftSpeed(rightTrigger  * MAX_SHOOTER_RPS);
                motors.setRightSpeed(rightTrigger * MAX_SHOOTER_RPS);
                motors.setIndexerSpeed(leftTrigger / 2.0); // Scale indexer slower than full trigger range.
            }, motors)
        );

        // Button edges flip sign of direction multipliers (runOnce = one-shot on press).
        joystick.square().onTrue(Commands.runOnce(() -> indexDirection  *= -1));
        joystick.cross().onTrue( Commands.runOnce(() -> indexDirection  *= -1));
        joystick.triangle().onTrue(Commands.runOnce(() -> shooterDirection *= -1));

        // Options: reset field-centric forward to current heading as 0°. L1: seed without forcing a specific rotation.
        joystick.options().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero))
        );
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // SysId routines for characterizing drive; hold modifier + face button while enabled.
        joystick.create().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.create().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Forward swerve state to AdvantageKit logger and to NetworkTables struct for external visualization.
        drivetrain.registerTelemetry(state -> {
            logger.telemeterize(state); // Record module states, pose, etc. via Telemetry class.

            m_reflectTelemetry.rotation       = state.Pose.getRotation();
            m_reflectTelemetry.currentSpeeds  = state.Speeds;
            m_reflectTelemetry.currentStates  = state.ModuleStates;
            m_reflectTelemetry.desiredStates  = state.ModuleTargets;
            m_telemetryPublisher.set(m_reflectTelemetry); // Push one struct sample per drivetrain update.
        });
    }

    // ── Autonomous ────────────────────────────────────────────────────────────

    public Command getAutonomousCommand() {
        final double AUTO_SHOOTER_RPS = 70.0;

        return Commands.sequence(
            // Example commented: timed drive segment could go here before shooter.
            Commands.run(() -> {
            System.out.print("auto"); // Debug: proves this runnable is active during auto.
            motors.setLeftSpeed(AUTO_SHOOTER_RPS);
            motors.setRightSpeed(AUTO_SHOOTER_RPS);
            motors.setIndexerSpeed(0.5);
        }, motors) // Requires shooter subsystem; runs forever until mode change or another command takes over.
        );
    }
    
    /**
     * Follow the "MyPath" Choreo trajectory.
     * Rename this method and the trajectory string to match each .traj file
     * you add to src/main/deploy/choreo/.
     */
     public void autonomousInit() {
        _autonomousCommand = autoChooser.selectedCommand(); // Build Command from chosen AutoRoutine name.
        if (_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(_autonomousCommand); // Start the auto command group.
        }
     }

    /**
     * Builds a Choreo auto routine named "leave" using trajectory file leave.traj (under deploy/choreo/).
     */
    private AutoRoutine MyPath(AutoFactory factory) {
        var routine = factory.newRoutine("leave"); // Logical name for this routine (can match traj name).
        var traj    = routine.trajectory("leave"); // Filename stem of the .traj in deploy/choreo/

        // When the routine becomes active, reset odometry to the path start then follow the trajectory.
        routine.active().onTrue(
            Commands.sequence(
                traj.resetOdometry(), // Set pose estimator to the first pose in the trajectory.
                traj.cmd()            // Command that plays through the trajectory via followChoreoSample.
            )
        );

        return routine;
    }

    //  private AutoRoutine MyPathPlanner(AutoFactory factory) {
    //     PathPlannerPath leave = PathPlanner.fromChoreoTrajectory("leave");
    //     var routine = factory.newRoutine("leave");
    //     var traj    = routine.trajectory("leave"); // must match MyPath.traj filename

    //     routine.active().onTrue(
    //         Commands.sequence(
    //             traj.resetOdometry(), // snap odometry to path start pose
    //             traj.cmd()            // follow the path
    //         )
    //     );

    //     return routine;
    // }

}
