package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Drivetrain;

public class LeadingTargetTracker extends TargetTracker {
    public LeadingTargetTracker(Drivetrain drivetrain) {
        super(drivetrain);
    }

    @Override
    public Rotation2d getRobotToTargetRelativeRotation() {
        return null;
    }
}