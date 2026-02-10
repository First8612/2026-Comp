package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StartShooter extends Command{
    private Shooter shooter;
    
    private PIDController shooterPID = new PIDController(0, 0, 0);

    // public StartShooter(double speed) {
    //     shooter.motorSpeedGoal = speed;
    // }
    
}