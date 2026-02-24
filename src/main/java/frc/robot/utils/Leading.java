package frc.robot.utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Field;
import frc.robot.Target;

//TODO: Implement leading into LeadingTargetTracker
public class Leading{
    private Translation2d leadingTrans = new Translation2d();
    private InterpolatingArrayTreeMap leadData;
    private Drivetrain drivetrain;
    private Translation2d AimPos;

    public Leading(InterpolatingArrayTreeMap treeMap, Translation2d startingPos, Drivetrain drivetrain) {
        leadData = treeMap;

        this.drivetrain = drivetrain;
    }

    public void calculateLead(double threshold, int loopLimit) {
        leadingTrans = new Translation2d(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
        double distOff = threshold + 1;
        Translation2d targetPos = Target.getPose().getTranslation();
        AimPos = targetPos;
        double hangTime;
        int eStop = loopLimit;
        while(distOff > threshold && eStop > 0) {
            hangTime = leadData.get(drivetrain.getState().Pose.getTranslation().getDistance(AimPos))[3];

            distOff = Math.abs(AimPos.getNorm() - targetPos.plus(leadingTrans.times(hangTime)).getNorm());
            AimPos = targetPos.plus(leadingTrans.times(hangTime));
            eStop --;
        }
    }

    public Translation2d getTarget() {
        return AimPos;
    }
}