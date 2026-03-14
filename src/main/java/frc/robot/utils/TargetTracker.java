
package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        State state = new State();

        // Alliance/target logic
        state.currentAllianceField = DriverStation.getAlliance()
            .map(color -> color == Alliance.Red ? Field.redAlliance : Field.blueAlliance)
            .orElse(Field.blueAlliance);

        if (state.currentAllianceField.zone.contains(robotPose.getTranslation())) {
            state.currentTarget = state.currentAllianceField.hub;
        } else {
            state.currentTarget = state.currentAllianceField.passingTargetRight;
        }

        // Directly compute target state
        Translation2d robotToTargetTranslation = state.currentTarget.getTranslation().minus(robotPose.getTranslation());
        Rotation2d robotToTargetDirection = robotToTargetTranslation.getAngle();

        state.robotToTargetRotation = robotToTargetDirection.rotateBy(drivetrain.getOperatorForwardDirection());
        state.robotToTargetRelativeRotation = robotToTargetTranslation.getAngle().minus(robotPose.getRotation());
        state.robotToTargetTranslation = robotToTargetTranslation;

        return state;
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
        // SmartDashboard.putNumber("Target/robotToTargetAbsoluteRotation", getRobotToTargetRotation().getDegrees());
        // SmartDashboard.putNumber("Target/robotToTargetRelativeRotation", getRobotToTargetRelativeRotation().getDegrees());
    }

    private static class State {
        public Field.ByAlliance currentAllianceField = Field.blueAlliance;
        public Pose2d currentTarget = currentAllianceField.hub;
        public Rotation2d robotToTargetRotation;
        public Rotation2d robotToTargetRelativeRotation;
        public Translation2d robotToTargetTranslation;
    }
}
