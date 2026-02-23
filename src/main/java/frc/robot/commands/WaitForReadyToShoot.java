package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TargetTracker;

public class WaitForReadyToShoot extends Command {
    private Debouncer yawDebounce = new Debouncer(.5, DebounceType.kRising);
    private Shooter shooter;
    private TargetTracker targetTracker;
    private boolean ignoreYaw;

    public WaitForReadyToShoot(Shooter shooter, TargetTracker targetTracker, boolean ignoreYaw) {
        super();
        this.shooter = shooter;
        this.targetTracker = targetTracker;
        this.ignoreYaw = ignoreYaw;
    }

    @Override
    public boolean isFinished() {
        var yawIsReady = ignoreYaw || yawDebounce.calculate(
                Math.abs(targetTracker.getRobotToTargetRelativeRotation().getDegrees()) < 2);
        var shooterIsReady = shooter.readyToShoot();

        return yawIsReady && shooterIsReady;
    }
}
