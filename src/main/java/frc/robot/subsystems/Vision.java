
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {

    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/Pose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> frontPoseMT1Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/Pose_WpiBlue/front", Pose2d.struct).publish();
    private StructPublisher<Pose2d> frontPoseMT2Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/Pose_MT2_WpiBlue/front", Pose2d.struct).publish();
    private StructPublisher<Pose2d> backPoseMT1Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/Pose_WpiBlue/back", Pose2d.struct).publish();
    private StructPublisher<Pose2d> backPoseMT2Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/Pose_MT2_WpiBlue/back", Pose2d.struct).publish();
    private Drivetrain driveBase;
    private boolean odometeryHasBeenReset;

    public Vision(Drivetrain drivebase) {

        super();

        driveBase = drivebase;
    }

    public void setOdometeryHasBeenReset(boolean hasBeenReset) {
        this.odometeryHasBeenReset = hasBeenReset;
    }

    public void periodic() {
        // periodicCamera("limelight-front", frontPoseMT1Publisher, frontPoseMT2Publisher);
        // periodicCamera("limelight-back", backPoseMT1Publisher, backPoseMT2Publisher);

        posePublisher.set(driveBase.getState().Pose);
    }

    private void periodicCamera(String limeLightName, StructPublisher<Pose2d> mt1Publisher,
            StructPublisher<Pose2d> mt2Publisher) {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightName);
        var stdevs = NetworkTableInstance.getDefault().getEntry(limeLightName + "/stddevs").getDoubleArray(new Double[] { 0.0, 0.0, 0.0});

        mt1Publisher.set(poseEstimate.pose);

        // MT2
        var useMT2 = odometeryHasBeenReset;
        LimelightHelpers.SetRobotOrientation(limeLightName,
                driveBase.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        var poseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);
        mt2Publisher.set(poseEstimateMT2.pose);

        if (useMT2) {
            poseEstimate = poseEstimateMT2;
        }

        if (poseEstimate.tagCount != 0) {
            var readingSetDevs = VecBuilder.fill(stdevs[0], stdevs[1], stdevs[2]);
            driveBase.setVisionMeasurementStdDevs(readingSetDevs);
            driveBase.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }

    }
}