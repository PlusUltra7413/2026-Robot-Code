package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class climber extends SubsystemBase {

    // --- Constants ---
    private static final int LEFT_CLIMBER_ID  = 20; // Change to your CAN IDs
    private static final String CANBUS = "rio"; // or your CANivore bus name

    private static final double CLIMB_SPEED  =  0.8;  // Power when climbing up
    private static final double LOWER_SPEED  = -0.5;  // Power when lowering arms
    private static final double HOLD_OUTPUT  =  0.00; // Small hold to fight gravity

    // Soft limit constants (in rotations of the motor shaft)
    // Tune these by printing leftMotor.getPosition() while manually moving the arm
    private static final double SOFT_LIMIT_FORWARD = 135.0; // max extension (rotations)
    private static final double SOFT_LIMIT_REVERSE = 0.0;   // fully retracted (rotations)

    // --- Hardware ---
    private final TalonFX leftMotor;

    // --- Control requests ---
    private final DutyCycleOut leftRequest  = new DutyCycleOut(0).withEnableFOC(true);

    public climber() {
        leftMotor  = new TalonFX(LEFT_CLIMBER_ID,  CANBUS);
        configMotor(leftMotor,  false);
    }

    private void configMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Hold position when stopped
        config.MotorOutput.Inverted    = inverted
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        // Current limits to protect your robot during climb
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit   = 40; // amps
        //config.CurrentLimits.SupplyCurrentThresholdTime = 0.1; // seconds

        // Soft limits — motor will stop commanding output beyond these positions
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_LIMIT_FORWARD;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_LIMIT_REVERSE;

        motor.getConfigurator().apply(config);
    }

    // --- Public methods ---


    public void climbUp() {setSpeed(CLIMB_SPEED);}
    public void climbDown() {setSpeed(LOWER_SPEED);}
    public void hold() {setSpeed(HOLD_OUTPUT);}
    public void stop() {setSpeed(0);}

    private void setSpeed(double output) {
        leftMotor.setControl(leftRequest.withOutput(output));
    }

    @Override
    public void periodic() {
        // You can log encoder positions here if needed
        SmartDashboard.putNumber("Climber/Left Position", leftMotor.getPosition().getValueAsDouble());
    }
}