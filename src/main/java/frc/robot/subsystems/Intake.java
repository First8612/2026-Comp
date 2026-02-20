package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotor; // = new TalonFX(10);
    private TalonFX extendMotor; // = new TalonFX(11);
    private String extendStatus = "Extended";
    
    private double fake_intakeSpeed = 0;

    public void in() {
        fake_intakeSpeed = 0.5;
    }

    public void out() {
        fake_intakeSpeed = -0.5;
    }

    public void setSpeedDutyCycle(double dutyCycle) {
        fake_intakeSpeed = dutyCycle;
    }

    public void stop() {
        fake_intakeSpeed = 0;
    }

    public void extend() {
        extendStatus = "Extending";
    }

    public void retract() {
        extendStatus = "Retracting";
    }

    public Boolean isExtended() {
        return extendStatus == "Extended";
    }

    public Boolean isRetracted() {
        return extendStatus == "Retracted";
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/speed", fake_intakeSpeed);
        SmartDashboard.putString("Intake/extendStatus", extendStatus);
    }
}
