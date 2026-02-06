package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake{
    TalonFX intakeMotor = new TalonFX(40);
    private double intakeSpeed = 0;

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }
}
