package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBuses;

public class Storage extends SubsystemBase{
    private TalonFX conveyMotor = new TalonFX(30, CANBuses.intake);
    private double conveyDutyCycle = 0;
    

    public Storage() {
        super();
        //More Init
        
        setDefaultCommand(Commands.runOnce(this::conveyStop, this));
    }

    public void conveyIn() {
        conveyDutyCycle = 0.75;
    }

    public void conveyOut() {
        conveyDutyCycle = -0.75;
    }

    public void conveyStop() {
        conveyDutyCycle = 0;
    }

    public boolean hasFuel() {
        return false;
    }

    @Override
    public void periodic() {
        conveyMotor.set(conveyDutyCycle);
    }
}
