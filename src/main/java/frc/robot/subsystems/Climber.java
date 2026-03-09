package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CANBuses;

public class Climber {
    private final TalonFX climbMotorRight = new TalonFX(0, CANBuses.intake);
    private final TalonFX climbMotorLeft = new TalonFX(0, CANBuses.intake);

    public Climber() {
        super();
    }
}
