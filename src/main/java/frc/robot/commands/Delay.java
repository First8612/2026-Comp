package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Delay extends ParallelCommandGroup{
    public Delay (){
        super();
    }
    public void setDelay(double delay) {
        this.addCommands(new WaitCommand(delay));
        this.addCommands(new InstantCommand(() -> SmartDashboard.putNumber("Delay/Auto Delay Used", delay)));
    }
}
