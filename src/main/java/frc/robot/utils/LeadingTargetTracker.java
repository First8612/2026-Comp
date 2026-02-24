package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Target;
import frc.robot.subsystems.Drivetrain;

public class LeadingTargetTracker extends TargetTracker {
    private Leading leader;
    private Drivetrain drivetrain;
    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Leading/Target", Pose2d.struct).publish();
    public LeadingTargetTracker(Drivetrain drivetrain) {
        super(drivetrain);
        this.drivetrain = drivetrain;
        leader = new Leading(drivetrain);
        leader.calculateLead(5);
    }

    @Override
    public Rotation2d getRobotToTargetRotation() {
        return leader.getDirectionFrom(drivetrain.getState().Pose)
                .rotateBy(drivetrain.getOperatorForwardDirection());
    }

    @Override
    public Rotation2d getRobotToTargetRelativeRotation() {
        var robotPose = drivetrain.getState().Pose;

        return leader.getTranslationFrom(robotPose)
            .getAngle().minus(robotPose.getRotation());
    }

    @Override
    public Translation2d getRobotToTargetTranslation() {
        return leader.getTranslationFrom(drivetrain.getState().Pose);
    }
    
    @Override
    public void periodic() {
        leader.calculateLead(5);
        SmartDashboard.putNumber("Leading/robotToTargetAbsoluteRotation", getRobotToTargetRotation().getDegrees());
        SmartDashboard.putNumber("Leading/robotToTargetRelativeRotation", getRobotToTargetRelativeRotation().getDegrees());
        posePublisher.set(leader.getTargetAsPose2d());
    }
}