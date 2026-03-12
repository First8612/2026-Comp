package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class ShootFuel extends Command {
    private Shooter shooter;
    private Storage storage;

    public ShootFuel(Shooter shooter, Storage storage) {
        super();
        this.shooter = shooter;
        this.storage = storage;

        addRequirements(shooter);
        addRequirements(storage);
    }

    @Override
    public void initialize() {
        shooter.enableFeeding();
        storage.conveyIn();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.enableFeeding(false);
        storage.conveyStop();
    }
}
