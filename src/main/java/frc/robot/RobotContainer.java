// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.lib.SwerveTelemetry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.climber;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //CHNAGE PID VALUES HERE
    private final PIDController headingController = new PIDController(0.05, 0.0, 0.001);
   
   
    private double targetHeadingRad = 0.0; // the heading we want to hold
    private static final double ROTATION_DEADBAND = 0.1; // same as your existing deadband
    
    private final Shooter motors = new Shooter();
   // private final climber climber = new climber();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private double indexDirection = 1.0;
    private double shooterDirection = 1.0;

    // --- Reflect Swerve Telemetry ---
    private final SwerveTelemetry m_reflectTelemetry = new SwerveTelemetry();
    private final StructPublisher<SwerveTelemetry> m_telemetryPublisher =
        NetworkTableInstance.getDefault()
            .getTable("Robot")
            .getStructTopic("SwerveTelemetry", SwerveTelemetry.struct)
            .publish();


    public RobotContainer() {
        headingController.enableContinuousInput(-Math.PI, Math.PI); // handles the -180/180 wrap
        configureBindings();

        drivetrain.seedFieldCentric(Rotation2d.kZero);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
            double rotInput = -joystick.getRightX() * MaxAngularRate;

            if (Math.abs(joystick.getRightX()) > ROTATION_DEADBAND) {
                targetHeadingRad = drivetrain.getState().Pose.getRotation().getRadians();
                return drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(rotInput);
            } else {
                double currentHeadingRad = drivetrain.getState().Pose.getRotation().getRadians();
                double correction = headingController.calculate(currentHeadingRad, targetHeadingRad);
                return drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(correction);
            }
        })
    );


        /** 
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        */
        /** 
            climber.setDefaultCommand(new RunCommand(() -> climber.hold(), climber));
        
        joystick.povUp().whileTrue(new RunCommand(() -> climber.climbUp(),   climber))
             .onFalse(Commands.runOnce(() -> climber.hold(),       climber));

        joystick.povDown().whileTrue(new RunCommand(() -> climber.climbDown(), climber))
             .onFalse(Commands.runOnce(() -> climber.hold(),       climber));
             **/
        joystick.square().onTrue(
    Commands.runOnce(() -> indexDirection *= -1)
);
        


// Toggle shooter direction safely
joystick.cross().onTrue(
    Commands.runOnce(() -> indexDirection *= -1)
);
joystick.triangle().onTrue(
    Commands.runOnce(() -> shooterDirection *= -1)
);

// Default command for shooter motors
motors.setDefaultCommand(
    new RunCommand(() -> {
        if (DriverStation.isJoystickConnected(0)) {
        // Controller is connected


        double leftTrigger = joystick.getL2Axis();
        double rightTrigger = joystick.getR2Axis();

        // Apply direction
        leftTrigger ++;
        rightTrigger++;
        leftTrigger *= indexDirection;
        rightTrigger *= shooterDirection;
        SmartDashboard.putNumber("indexer", indexDirection);
        SmartDashboard.putNumber("shooter", shooterDirection);
        rightTrigger *= 1; 

        double MAX_SHOOTER_RPS = 80.0; // tune this to your desired top speed (0-100)

        motors.setLeftSpeed(rightTrigger * MAX_SHOOTER_RPS);
        motors.setRightSpeed(rightTrigger * MAX_SHOOTER_RPS);
        motors.setIndexerSpeed(leftTrigger / 2); 

        }
    }, motors)
);                                                    


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
       

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        joystick.options().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)));


        
        joystick.create().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.create().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(state -> {
        // AdvantageKit
            logger.telemeterize(state);

        // Reflect
        m_reflectTelemetry.rotation = state.Pose.getRotation();
        m_reflectTelemetry.currentSpeeds = state.Speeds;
        m_reflectTelemetry.currentStates = state.ModuleStates;
        m_reflectTelemetry.desiredStates = state.ModuleTargets;
        m_telemetryPublisher.set(m_reflectTelemetry);
});
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
