package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;




public class Shooter extends SubsystemBase {

    private final TalonFX leftMotor = new TalonFX(11);
    private final TalonFX rightMotor = new TalonFX(10);

    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);




    public Shooter() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2; 

        leftMotor.getConfigurator().apply(config);


        // Invert the right motor
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(config);
        }

    private double applyDeadband(double value) {
    return Math.abs(value) < 0.05 ? 0.0 : value;
    }

    private double clamp(double value) {
    return Math.max(-1.0, Math.min(1.0, value));
    }

    public void setLeftSpeed(double speed) {
    speed = applyDeadband(speed);
    leftOut.Output = speed;
    leftMotor.setControl(leftOut);
    }

    public void setRightSpeed(double speed) {
    speed = applyDeadband(speed);
    rightOut.Output = speed;
    rightMotor.setControl(rightOut);
    }


    public void stop() {
        setLeftSpeed(0);
        setRightSpeed(0);
    }
}

