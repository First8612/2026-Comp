package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SafeRobotForTrench extends Command {
    private Intake intake;
    private Shooter shooter;

    public SafeRobotForTrench(Intake intake, Shooter shooter) {
        super();
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        intake.extend();
        shooter.stop();
    }
}
