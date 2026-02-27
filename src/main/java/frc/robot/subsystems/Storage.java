package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase{
    private TalonFX conveyMotor = new TalonFX(30,"Intake");
    private double test_conveySpeed = 0;
    

    public Storage() {
        super();
        //More Init

        setDefaultCommand(Commands.runOnce(this::conveyStop, this));
    }

    public void conveyIn() {
        test_conveySpeed = 0.5;
    }

    public void conveyOut() {
        test_conveySpeed = -0.5;
    }

    public void conveyStop() {
        test_conveySpeed = 0;
    }

    public boolean hasFuel() {
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Storage/conveySpeed", test_conveySpeed);
        SmartDashboard.putBoolean("Storage/hasFuel", hasFuel());
    }
}
