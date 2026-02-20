package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Target;
import frc.robot.subsystems.Drivetrain;

public class TargetTracker {
    private Drivetrain drivetrain;

    public TargetTracker(Drivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
    }

    public Rotation2d getRobotToTargetRotation() {
        return Target.getDirectionFrom(drivetrain.getState().Pose)
                .rotateBy(drivetrain.getOperatorForwardDirection());
    }

    public Translation2d getRobotToTargetTranslation() {
        return Target.getTranslationFrom(drivetrain.getState().Pose);
    }

}
