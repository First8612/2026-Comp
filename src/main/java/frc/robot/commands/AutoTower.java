package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class AutoTower extends Command {
    private Drivetrain drivetrain;
    private Pose2d nearestAlignSpot;
    private final SwerveRequest.FieldCentricFacingAngle driveRequest = 
        new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(10,0,0)
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    
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

        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    @Override
    public void execute() {
        var robotPose = drivetrain.getCachedState();

        var translation = robotPose.Pose.getTranslation().minus(
              nearestAlignSpot.getTranslation()
        );

        driveRequest
            .withVelocityX(
                MathUtil.clamp(xController.calculate(translation.getX()), 2, 2) * RobotContainer.MaxSpeed
            )
            ;
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
