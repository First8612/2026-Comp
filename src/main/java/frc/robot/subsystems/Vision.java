
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {

    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/Pose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> frontPoseMT1Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/front/MT1", Pose2d.struct).publish();
    private StructPublisher<Pose2d> frontPoseMT2Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/front/MT2", Pose2d.struct).publish();
    private DoublePublisher frontStdDevPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Poses/front/StdDev").publish();
    private StructPublisher<Pose2d> backPoseMT1Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/back/MT1", Pose2d.struct).publish();
    private StructPublisher<Pose2d> backPoseMT2Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/back/MT2", Pose2d.struct).publish();
    private DoublePublisher backStdDevPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Poses/back/StdDev").publish();
    private BooleanPublisher useMT2Publisher = NetworkTableInstance.getDefault()
            .getBooleanTopic("Poses/useMT2").publish();
    private DoublePublisher frontLatencyPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Poses/front/latency").publish();
    private DoublePublisher backLatencyPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Poses/back/latency").publish();
    private Drivetrain driveBase;
    private Boolean useMT2 = false;
    private long mt1Readings = 0;

    public Vision(Drivetrain drivebase) {

        super();

        this.driveBase = drivebase;
    }

    public void reset() {
        this.useMT2 = false;
        mt1Readings = 0;
    }

    public void periodic() {
        handleLimelight("limelight-front", frontPoseMT1Publisher, frontPoseMT2Publisher, frontStdDevPublisher, frontLatencyPublisher);
        handleLimelight("limelight-back", backPoseMT1Publisher, backPoseMT2Publisher, backStdDevPublisher, backLatencyPublisher);

        // posePublisher.set(driveBase.getCachedState().Pose);
        // useMT2Publisher.set(useMT2);
    }

    private void handleLimelight(
        String limelightName, 
        StructPublisher<Pose2d> mt1Publisher, 
        StructPublisher<Pose2d> mt2Publisher, 
        DoublePublisher stdDevPublisher,
        DoublePublisher latencyPublisher

    ) {
        var drivetrainState = driveBase.getCachedState();
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        // mt1Publisher.set(poseEstimate.pose);

        if (!useMT2) {
            if (poseEstimate.tagCount != 0) {
                mt1Readings ++;

                if (mt1Readings > 100) {
                    useMT2 = true;
                }
            }
        }
        else {
            LimelightHelpers.SetRobotOrientation(limelightName,
                    drivetrainState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            // mt2Publisher.set(poseEstimate.pose);
        }

        var latency = drivetrainState.Timestamp - poseEstimate.timestampSeconds;
        // latencyPublisher.set(latency);


        if (poseEstimate.tagCount != 0) {

            // stolen from https://github.com/Enigma2075/FRC2025/blob/05b738aa4bcf2dd822304b07f8c74f97dc0b25d0/src/main/java/frc/robot/subsystems/Vision.java#L287-L297
            double stdDevFactor = Math.pow(poseEstimate.avgTagDist, 2.0) / poseEstimate.tagCount;
            double linearStdDev = 0.75 * stdDevFactor * (Math.pow(drivetrainState.Speeds.omegaRadiansPerSecond, 2) + 1);
            double angularStdDev = 999999999 * stdDevFactor;
            // stdDevPublisher.set(linearStdDev);

            driveBase.setVisionMeasurementStdDevs(VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            driveBase.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }

    }
}