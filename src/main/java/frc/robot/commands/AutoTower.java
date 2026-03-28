package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Field;
import frc.robot.subsystems.Drivetrain;

public class AutoTower extends Command {
    private Drivetrain drivetrain;
    private Pose2d nearestAlignSpot;
    private PIDController xController = new PIDController(0.01, 0, 0);
    private PIDController yController = new PIDController(0.01, 0, 0);
    private PIDController rotationController = new PIDController(0.01, 0, 0);

    public AutoTower(Drivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        var robotPose = drivetrain.getCachedState();
        var alliance = Field.getMyAlliance();

        nearestAlignSpot = robotPose.Pose.nearest(Arrays.asList(
            alliance.towerAlignLeft,
            alliance.towerAlignRight
        ));
    }

    @Override
    public void execute() {
        var robotPose = drivetrain.getCachedState();

        // TODO
    }









    public static Command get(Drivetrain drivetrain) {
    

        var robotPose = drivetrain.getCachedState();
        var alliance = Field.getMyAlliance();

        var nearestAlignSpot = robotPose.Pose.nearest(Arrays.asList(
            alliance.towerAlignLeft,
            alliance.towerAlignRight
        ));

        var constraints = new PathConstraints(3.0, 3.0, 4.0, 4.0);

        // This generates a path from your current location to the target
        return AutoBuilder.pathfindToPose(
            nearestAlignSpot, 
            constraints, 
            0.0 // Goal end velocity
        );
    }
}
