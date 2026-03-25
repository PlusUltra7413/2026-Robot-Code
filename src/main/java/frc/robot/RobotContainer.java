// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.lib.SwerveTelemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    // ── Speeds ────────────────────────────────────────────────────────────────
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // ── Heading hold PID (tune kP / kD as needed) ─────────────────────────────
    private final PIDController headingController = new PIDController(0.05, 0.0, 0.001);
    private double targetHeadingRad = 0.0;
    private static final double ROTATION_DEADBAND = 0.1;

    // ── Subsystems ────────────────────────────────────────────────────────────
    private final Shooter motors = new Shooter();
    // private final climber climber = new climber();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // ── Swerve requests ───────────────────────────────────────────────────────
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    // ── Telemetry ─────────────────────────────────────────────────────────────
    public final Telemetry logger = new Telemetry(MaxSpeed);
    private final SwerveTelemetry m_reflectTelemetry = new SwerveTelemetry();
    private final StructPublisher<SwerveTelemetry> m_telemetryPublisher =
        NetworkTableInstance.getDefault()
            .getTable("Robot")
            .getStructTopic("SwerveTelemetry", SwerveTelemetry.struct)
            .publish();

    // ── Controller ────────────────────────────────────────────────────────────
    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    // ── Shooter direction state ───────────────────────────────────────────────
    private double indexDirection = 1.0;
    private double shooterDirection = 1.0;

    // ── Choreo auto ───────────────────────────────────────────────────────────
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    // ─────────────────────────────────────────────────────────────────────────
    public RobotContainer() {
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        // Build the Choreo factory BEFORE configureBindings so the drivetrain
        // subsystem is ready when routines are registered.
        autoFactory = new AutoFactory(
            () -> drivetrain.getState().Pose,   // current pose supplier
            drivetrain::resetPose,              // pose reset consumer
            drivetrain::followChoreoSample,     // sample follower
            true,                               // mirror paths on red alliance
            drivetrain                          // subsystem requirement
        );

        autoChooser = new AutoChooser();
        // ── Add more routines here as you create .traj files in deploy/choreo/ ──
        autoChooser.addRoutine("MyPath", () -> myPathAuto(autoFactory));

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();

        drivetrain.seedFieldCentric(Rotation2d.kZero);
    }

    // ─────────────────────────────────────────────────────────────────────────
    private void configureBindings() {

        // ── Default drive command with heading-hold ───────────────────────────
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double rotInput = -joystick.getRightX() * MaxAngularRate;

                if (Math.abs(joystick.getRightX()) > ROTATION_DEADBAND) {
                    // Driver is actively rotating — track current heading so we
                    // don't snap back when they let go.
                    targetHeadingRad = drivetrain.getState().Pose.getRotation().getRadians();
                    return drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(rotInput);
                } else {
                    // No rotation input — hold the last heading with PID.
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

        // ── Disable idle ──────────────────────────────────────────────────────
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // ── Shooter / indexer defaults ────────────────────────────────────────
        motors.setDefaultCommand(
            new RunCommand(() -> {
                if (!DriverStation.isJoystickConnected(0)) return;

                double leftTrigger  = joystick.getL2Axis() + 1.0; // remap [-1,1] → [0,2]
                double rightTrigger = joystick.getR2Axis() + 1.0;

                leftTrigger  *= indexDirection;
                rightTrigger *= shooterDirection;

                SmartDashboard.putBoolean("indexer", indexDirection == 1.0);
                SmartDashboard.putBoolean("shooter", shooterDirection == 1.0);

                final double MAX_SHOOTER_RPS = 80.0;
                motors.setLeftSpeed(rightTrigger  * MAX_SHOOTER_RPS);
                motors.setRightSpeed(rightTrigger * MAX_SHOOTER_RPS);
                motors.setIndexerSpeed(leftTrigger / 2.0);
            }, motors)
        );

        // ── Shooter/indexer direction toggles ─────────────────────────────────
        joystick.square().onTrue(Commands.runOnce(() -> indexDirection  *= -1));
        joystick.cross().onTrue( Commands.runOnce(() -> indexDirection  *= -1));
        joystick.triangle().onTrue(Commands.runOnce(() -> shooterDirection *= -1));

        // ── Field-centric reset ───────────────────────────────────────────────
        joystick.options().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero))
        );
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // ── SysId (hold Create + face button) ────────────────────────────────
        joystick.create().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.create().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ── Telemetry ─────────────────────────────────────────────────────────
        drivetrain.registerTelemetry(state -> {
            logger.telemeterize(state);

            m_reflectTelemetry.rotation       = state.Pose.getRotation();
            m_reflectTelemetry.currentSpeeds  = state.Speeds;
            m_reflectTelemetry.currentStates  = state.ModuleStates;
            m_reflectTelemetry.desiredStates  = state.ModuleTargets;
            m_telemetryPublisher.set(m_reflectTelemetry);
        });
    }

    // ── Autonomous ────────────────────────────────────────────────────────────

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommandScheduler();
    }

    /**
     * Follow the "MyPath" Choreo trajectory.
     * Rename this method and the trajectory string to match each .traj file
     * you add to src/main/deploy/choreo/.
     */
    private AutoRoutine myPathAuto(AutoFactory factory) {
        var routine = factory.newRoutine("MyPath");
        var traj    = routine.trajectory("MyPath"); // must match MyPath.traj filename

        routine.active().onTrue(
            Commands.sequence(
                traj.resetOdometry(), // snap odometry to path start pose
                traj.cmd()            // follow the path
            )
        );

        return routine;
    }
}
