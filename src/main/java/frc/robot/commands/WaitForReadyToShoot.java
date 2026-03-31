package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TargetTracker;

public class WaitForReadyToShoot extends Command {
    private Debouncer yawDebounce = new Debouncer(.5, DebounceType.kRising);
    private Shooter shooter;
    private TargetTracker targetTracker;
    private boolean ignoreYaw;
    private BooleanSupplier hasClimbedSupplier;

    public WaitForReadyToShoot(Shooter shooter, TargetTracker targetTracker, boolean ignoreYaw, BooleanSupplier hasClimbedSupplier) {
        super();
        this.shooter = shooter;
        this.targetTracker = targetTracker;
        this.ignoreYaw = ignoreYaw;
        this.hasClimbedSupplier = hasClimbedSupplier;
    }

    @Override
    public boolean isFinished() {
        var yawIsReady = 
                ignoreYaw 
                || hasClimbedSupplier.getAsBoolean()
                || yawDebounce.calculate(
                    Math.abs(targetTracker.getRobotToTargetRelativeRotation().getDegrees()) < 3
                ) 
                || targetTracker.getRobotToTargetTranslation().getNorm() > 100;
                
        var shooterIsReady = shooter.readyToShoot();

        return yawIsReady && shooterIsReady;
    }
}
