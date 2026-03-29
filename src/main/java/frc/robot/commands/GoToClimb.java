package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Field;
import frc.robot.subsystems.Drivetrain;

public class GoToClimb extends SequentialCommandGroup {
    public GoToClimb(Drivetrain drivetrain) {
        super();

        var robotPose = drivetrain.getCachedState();
        var alliance = Field.getMyAlliance();

        var nearestAlignSpot = robotPose.Pose.nearest(Arrays.asList(
            alliance.towerAlignLeft,
            alliance.towerAlignRight
        ));
        var climbPose = nearestAlignSpot.plus(new Transform2d(
            Meters.of(-0.3), 
            Meters.of(0), 
            Rotation2d.kZero));

        var constraints = new PathConstraints(3.0, 3.0, 4.0, 4.0);
        var driveToSpot = AutoBuilder.pathfindToPose(
            nearestAlignSpot, 
            constraints, 
            0.0 // Goal end velocity
        );
        
        addCommands(driveToSpot);
        addCommands(new DriveToLocation(drivetrain, () -> nearestAlignSpot));
        addCommands(new DriveToLocation(drivetrain, () -> climbPose));

    }
}
