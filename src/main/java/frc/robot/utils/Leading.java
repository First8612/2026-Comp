package frc.robot.utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Field;
import frc.robot.Target;

//TODO: Implement leading into LeadingTargetTracker
public class Leading{
    private Translation2d leadingTrans = new Translation2d();
    private InterpolatingDoubleTreeMap leadData;
    private Drivetrain drivetrain;
    private Translation2d AimPos;

    public Leading(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        leadData = new InterpolatingDoubleTreeMap();
        leadData.put(0.0, 0.5);
        leadData.put(5.0, 1.0);
    }

    public void calculateLead(int loopLimit) {
        leadingTrans = new Translation2d(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond).rotateBy(drivetrain.getState().RawHeading).times(-1);
        Translation2d targetPos = Target.getPose().getTranslation();
        AimPos = targetPos;
        double hangTime;
        for(int i = 0; i < loopLimit; i++) {
            hangTime = leadData.get(drivetrain.getState().Pose.getTranslation().getDistance(AimPos));

            AimPos = targetPos.plus(leadingTrans.times(hangTime));
        }
    }

    public Translation2d getTarget() {
        return AimPos;
    }
    public Pose2d getTargetAsPose2d() {
        return new Pose2d(AimPos, new Rotation2d());
    }

    public Translation2d getTranslationFrom(Pose2d pose) {

        return AimPos.minus(pose.getTranslation());
    }

    public Rotation2d getDirectionFrom(Pose2d pose) {
        return getTranslationFrom(pose).getAngle();
    }
}