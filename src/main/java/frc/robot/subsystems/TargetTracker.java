
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Field;

public class TargetTracker extends SubsystemBase {
    private Drivetrain drivetrain;
    private State state;
    private StructPublisher<Pose2d> targetLocation = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/Target/location", Pose2d.struct).publish();

    public TargetTracker(Drivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
        this.state = new State();

        updateState();
    }

    private void updateState() {
        var robotPose = drivetrain.getCachedState().Pose;

        // Alliance/target logic
        state.currentAllianceField = Field.getMyAlliance();

        if (state.currentAllianceField.zone.contains(robotPose.getTranslation())) {
            state.currentTarget = state.currentAllianceField.hub;
            state.targetIsHub = true;
        } else {
            state.currentTarget = robotPose.nearest(List.of(
                state.currentAllianceField.passingTargetRight,
                state.currentAllianceField.passingTargetLeft
            ));
            state.targetIsHub = true;
        }

        // Directly compute target state
        Translation2d robotToTargetTranslation = state.currentTarget.getTranslation().minus(robotPose.getTranslation());
        Rotation2d robotToTargetDirection = robotToTargetTranslation.getAngle();

        state.robotToTargetRotation = robotToTargetDirection.rotateBy(drivetrain.getOperatorForwardDirection());

        var relativeRotations = robotToTargetTranslation.getAngle().minus(robotPose.getRotation()).getRotations() % 1.0;
        state.robotToTargetRelativeRotation = Rotation2d.fromRotations(relativeRotations);
        state.robotToTargetTranslation = robotToTargetTranslation;
    }

    public Rotation2d getRobotToTargetRotation() {
        return state.robotToTargetRotation;
    }

    public Rotation2d getRobotToTargetRelativeRotation() {
        return state.robotToTargetRelativeRotation;
    }

    public Translation2d getRobotToTargetTranslation() {
        return state.robotToTargetTranslation;
    }

    public boolean getTargetIsHub() {
        return state.targetIsHub;
    }

    @Override
    public void periodic() {
        super.periodic();
        updateState();

        targetLocation.set(state.currentTarget);
        SmartDashboard.putString("Target/alliance", state.currentAllianceField.name);
        SmartDashboard.putNumber("Target/robotToTargetAbsoluteRotation", getRobotToTargetRotation().getRotations());
        SmartDashboard.putNumber("Target/robotToTargetRelativeRotation", getRobotToTargetRelativeRotation().getRotations());
    }

    private static class State {
        public Field.ByAlliance currentAllianceField = Field.blueAlliance;
        public Pose2d currentTarget = currentAllianceField.hub;
        public Rotation2d robotToTargetRotation;
        public Rotation2d robotToTargetRelativeRotation;
        public Translation2d robotToTargetTranslation;
        public boolean targetIsHub;
    }
}
