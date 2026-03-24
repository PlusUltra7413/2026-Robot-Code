package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;




public class Shooter extends SubsystemBase {

    private final TalonFX leftMotor = new TalonFX(21);
    private final TalonFX rightMotor = new TalonFX(20);
    private final TalonFX indexer = new TalonFX(22);

    private final VelocityVoltage leftVelocity = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage rightVelocity = new VelocityVoltage(0).withSlot(0);


    // private final DutyCycleOut leftOut = new DutyCycleOut(0); // old code with no PID
    // private final DutyCycleOut rightOut = new DutyCycleOut(0);

    private final DutyCycleOut indexout = new DutyCycleOut(0);




    public Shooter() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // PID + Feedforward gains — tune these for your shooter
        // kS: voltage to overcome static friction
        // kV: voltage per RPS (1 / free speed in RPS is a good starting point)
        // kP: how aggressively to correct velocity errorconfig.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;
        config.Slot0.kP = 0.2;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // inverts right motor
        rightMotor.getConfigurator().apply(config);

       // config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2; 
        //leftMotor.getConfigurator().apply(config);
        }

    private double applyDeadband(double value) {
    return Math.abs(value) < 0.05 ? 0.0 : value;
    }

    private double clamp(double value) {
    return Math.max(-1.0, Math.min(1.0, value));
    }

    public void setLeftSpeed(double rps) {
        rps = applyDeadband(rps);
        leftMotor.setControl(leftVelocity.withVelocity(rps));
    
    //speed = applyDeadband(speed);
    //leftOut.Output = speed;
    //leftMotor.setControl(leftOut);
    }

    public void setRightSpeed(double rps) {
        rps = applyDeadband(rps);
        rightMotor.setControl(rightVelocity.withVelocity(rps));
    
    //speed = applyDeadband(speed);
    //rightOut.Output = speed;
    //rightMotor.setControl(rightOut);
    }
    public void setIndexerSpeed(double speed) {
    speed = applyDeadband(speed);
    indexout.Output = speed;
    indexer.setControl(indexout);
    }


    public void stop() {
        setLeftSpeed(0);
        setRightSpeed(0);
    }
}

