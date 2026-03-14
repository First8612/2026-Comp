package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Target;
import frc.robot.subsystems.Drivetrain;

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

        state.robotToTargetRotation = Target.getDirectionFrom(robotPose)
            .rotateBy(drivetrain.getOperatorForwardDirection());

        state.robotToTargetRelativeRotation = Target.getTranslationFrom(robotPose)
            .getAngle().minus(robotPose.getRotation());

        state.robotToTargetTranslation = Target.getTranslationFrom(robotPose);

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

    public void periodic() {
        this.state = getCurrentState();

        SmartDashboard.putNumber("Target/robotToTargetAbsoluteRotation", getRobotToTargetRotation().getDegrees());
        SmartDashboard.putNumber("Target/robotToTargetRelativeRotation", getRobotToTargetRelativeRotation().getDegrees());
    }


    private static class State {
        public Rotation2d robotToTargetRotation;
        public Rotation2d robotToTargetRelativeRotation;
        public Translation2d robotToTargetTranslation;
    }
}
