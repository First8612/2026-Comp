
package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Field;

public class TargetTracker extends SubsystemBase {
    private Drivetrain drivetrain;
    private State state;

    public TargetTracker(Drivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
        this.state = getCurrentState();
    }

    private State getCurrentState() {
        var robotPose = drivetrain.getCachedState().Pose;
        State newState = new State();

        // Alliance/target logic
        newState.currentAllianceField = DriverStation.getAlliance()
            .map(color -> color == Alliance.Red ? Field.redAlliance : Field.blueAlliance)
            .orElse(Field.blueAlliance);

        if (newState.currentAllianceField.zone.contains(robotPose.getTranslation())) {
            newState.currentTarget = newState.currentAllianceField.hub;
        } else {
            newState.currentTarget = newState.currentAllianceField.passingTargetRight;
        }

        // Directly compute target state
        Translation2d robotToTargetTranslation = newState.currentTarget.getTranslation().minus(robotPose.getTranslation());
        Rotation2d robotToTargetDirection = robotToTargetTranslation.getAngle();

        newState.robotToTargetRotation = robotToTargetDirection.rotateBy(drivetrain.getOperatorForwardDirection());

        var relativeRotations = robotToTargetTranslation.getAngle().minus(robotPose.getRotation()).getRotations() % 1.0;

        newState.robotToTargetRelativeRotation = Rotation2d.fromRotations(relativeRotations);

        newState.robotToTargetTranslation = robotToTargetTranslation;

        return newState;
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

    @Override
    public void periodic() {
        super.periodic();
        this.state = getCurrentState();
        SmartDashboard.putNumber("Target/robotToTargetAbsoluteRotation", getRobotToTargetRotation().getRotations());
        SmartDashboard.putNumber("Target/robotToTargetRelativeRotation", getRobotToTargetRelativeRotation().getRotations());
    }

    private static class State {
        public Field.ByAlliance currentAllianceField = Field.blueAlliance;
        public Pose2d currentTarget = currentAllianceField.hub;
        public Rotation2d robotToTargetRotation;
        public Rotation2d robotToTargetRelativeRotation;
        public Translation2d robotToTargetTranslation;
    }
}
