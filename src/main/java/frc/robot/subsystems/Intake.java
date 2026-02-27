package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DixieHornCommand;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(10,"Intake");
    private final TalonFX intakeExtendLeft = new TalonFX(11,"Intake");
    private final TalonFX intakeExtendRight = new TalonFX(12,"Intake");
    private double speed = 0;

    public Intake() {
        super();
        
        setDefaultCommand(Commands.runOnce(this::stop, this));
        DixieHornCommand.enrollSubsystemMotors(this, intakeMotor);
    }

    public void in() {
        speed = 0.75;
    }

    public void out() {
        speed = -0.75;
    }

    // intended for prototyping time
    public void setSpeedRaw(double intakeSpeed) {
        this.speed = intakeSpeed;
    }

    public void stop() {
        speed = 0;
    }

    public void extend() {
    }

    public void retract() {
    }

    public Boolean isExtended() {
        return true;
    }

    public Boolean isRetracted() {
        return false;
    }

    @Override
    public void periodic() {
        intakeMotor.set(speed);

        SmartDashboard.putNumber("Intake/speed", speed);
        SmartDashboard.putBoolean("Intake/extended", isExtended());
    }
}
