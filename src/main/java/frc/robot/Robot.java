// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// AdvantageKit: logging framework that records robot data to WPILOG and/or replays from a log.
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot; // WPILib TimedRobot subclass that hooks Logger into the main loop.
import org.littletonrobotics.junction.Logger; // Central API for metadata, receivers, and recorded outputs.
import org.littletonrobotics.junction.networktables.NT4Publisher; // Streams log data to NetworkTables (e.g. AdvantageScope live).
import org.littletonrobotics.junction.wpilog.WPILOGReader; // Reads a .wpilog file as the data source during replay.
import org.littletonrobotics.junction.wpilog.WPILOGWriter; // Writes recorded data to a .wpilog file on disk.

// CTRE: when using Phoenix Hoot logs, this reapplies timestamp and joystick streams during replay.
import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Starts the default USB camera stream for the driver station / dashboard.
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot; // (Imported by WPILib patterns; LoggedRobot extends similar lifecycle.)

/**
 * Top-level robot class. Extends {@link LoggedRobot} so AdvantageKit can log in both real and sim/replay modes.
 * Lifecycle: constructor → robotPeriodic every loop; mode callbacks (disabled, auto, teleop, test) on transitions.
 */
public class Robot extends LoggedRobot {
    // Holds the command selected for autonomous so teleop can cancel it when the match phase changes.
    private Command m_autonomousCommand;
    // Owns subsystems, button bindings, default commands, and auto routine wiring.
    private final RobotContainer m_robotContainer;

    // Replays CTRE Hoot timestamp + joystick data when playing back logs that include them.
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {

        // USB camera: publishes video to the dashboard (and DS) using the first detected camera.
        CameraServer.startAutomaticCapture();

        // Strings attached to the log file for identification in AdvantageScope / log tools.
        Logger.recordMetadata("ProjectName", "ScourgeOfTheSeas");
        Logger.recordMetadata("TeamNumber", "7413");

        if (isReal()) {
            // On the roboRIO: write logs (e.g. to USB at /U/logs if present) and publish live NT4 streams.
            Logger.addDataReceiver(new WPILOGWriter()); // logs to USB stick at /U/logs when USB is inserted
            Logger.addDataReceiver(new NT4Publisher()); // live data to AdvantageScope over NetworkTables
        } else {
            // Simulation / replay: optionally disable real-time pacing and feed Logger from a replay file.
            setUseTiming(false); // Replay as fast as possible or follow log timing depending on AdvantageKit setup.
            String logPath = LogFileUtil.findReplayLog(); // Locates the .wpilog to replay (env / deploy path).
            Logger.setReplaySource(new WPILOGReader(logPath)); // Drive robot state from recorded inputs.
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Optional output log from sim.
        }

        // After start(), do not add more data receivers — logging pipeline is sealed.
        Logger.start();

        // Construct bindings, subsystems, and default commands (see RobotContainer).
        m_robotContainer = new RobotContainer();
        // Below this voltage the roboRIO may brown out; lowering raises tolerance (use with care).
        RobotController.setBrownoutVoltage(7);
    }

    @Override
    public void robotPeriodic() {
        System.out.print("robot perodic"); // Debug print every robot loop (consider removing for competition).
        // Camera is started once in the constructor; avoid starting again every periodic.
        m_timeAndJoystickReplay.update(); // Advance CTRE replay helpers if in replay mode.
        CommandScheduler.getInstance().run(); // Runs default commands, schedules, and executes active commands.
        m_robotContainer.logger.telemLog(); // Pushes any buffered telemetry from the Telemetry helper.
        Logger.recordOutput("testvalue", 1234.0); // Example logged output visible in AdvantageScope.
    }

    @Override
    public void disabledInit() {} // Called once when entering disabled (between modes or after enable drops).

    @Override
    public void disabledPeriodic() {} // Called repeatedly while disabled.

    @Override
    public void disabledExit() {} // Called once when leaving disabled (e.g. before auto or teleop).

    @Override
    public void autonomousInit() {
        // Auto command selection and scheduling live in RobotContainer (Choreo AutoChooser pattern).
        m_robotContainer.autonomousInit();
        // Alternative: schedule a Command returned by getAutonomousCommand() here instead of delegating.
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // if (m_autonomousCommand != null) {
        //     System.out.print("auto schedule");
        //     CommandScheduler.getInstance().schedule(m_autonomousCommand);
        // }
    }

    @Override
    public void autonomousPeriodic() {
        System.out.print("autoPerodic"); // Debug print every autonomous loop.
    }

    @Override
    public void autonomousExit() {} // Called once when autonomous ends (before teleop or disabled).

    @Override
    public void teleopInit() {
        // Ensure the autonomous command does not keep running after the sandstorm/auto phase ends.
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {} // Called repeatedly during teleop (often empty if everything is command-based).

    @Override
    public void teleopExit() {} // Called once when leaving teleop.

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll(); // Test mode: clear any running commands for a clean slate.
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {} // Extra hook when running in simulation (physics may run elsewhere).
}
