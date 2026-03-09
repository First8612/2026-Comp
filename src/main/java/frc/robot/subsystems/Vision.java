
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase{

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

    public Vision(Drivetrain drivebase) {

        super();

        driveBase = drivebase;
    }

    public void periodic() {
        PoseEstimate poseEstimateFront = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
        PoseEstimate poseEstimateBack = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

        LimelightHelpers.SetRobotOrientation("limelight-front",
                driveBase.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-back",
                driveBase.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
            
        PoseEstimate poseEstimateFrontMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        PoseEstimate poseEstimateBackMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

        if(poseEstimateFrontMT2.tagCount != 0){
            driveBase.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            driveBase.addVisionMeasurement(poseEstimateFrontMT2.pose, poseEstimateFrontMT2.timestampSeconds);
        }
        
        // if(poseEstimateBackMT2.tagCount != 0){
        //     driveBase.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        //     driveBase.addVisionMeasurement(poseEstimateBackMT2.pose, poseEstimateBackMT2.timestampSeconds);
        // }

        posePublisher.set(driveBase.getState().Pose);
        frontPoseMT1Publisher.set(poseEstimateFront.pose);
        frontPoseMT2Publisher.set(poseEstimateFrontMT2.pose);
        backPoseMT1Publisher.set(poseEstimateBack.pose);
        backPoseMT2Publisher.set(poseEstimateBackMT2.pose);
    }
}