package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Field;
import frc.robot.subsystems.Drivetrain;

/**
 * Garauntees the robot will drive directly toward the entrance of the nearest trench, and through the middle of it.
 * Will take away rotational and side-to-side (Y) control, giving the driver only forward/back control.
 */
public class DriveTrenchRun extends Command {
    private Drivetrain drivetrain;
    private Supplier<SwerveRequest.FieldCentric> driveRequestSupplier;
    private Pose2d entrance = Pose2d.kZero;
    private Pose2d exit = Pose2d.kZero;
    private List<Field.Trench> trenches = List.of(Field.blueAlliance.trenchLeft, Field.blueAlliance.trenchRight, Field.redAlliance.trenchLeft, Field.redAlliance.trenchRight);
    private PIDController inTrenchYVelController = new PIDController(8, 10, 0);
    private Rotation2d robotTargetDirection = Rotation2d.kZero;

    private StructPublisher<Pose2d> entrancePublisher = NetworkTableInstance.getDefault().getStructTopic("Commands/DriveTrenchRun/entrance", Pose2d.struct).publish();
    private StructPublisher<Pose2d> exitPublisher = NetworkTableInstance.getDefault().getStructTopic("Commands/DriveTrenchRun/exit", Pose2d.struct).publish();
    private DoublePublisher yVelPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Commands/DriveTrenchRun/yVel").publish();

    public DriveTrenchRun(Drivetrain drivetrain, Supplier<SwerveRequest.FieldCentric> driveRequestSupplier) {
        super();
        this.drivetrain = drivetrain;
        this.driveRequestSupplier = driveRequestSupplier;
        inTrenchYVelController.setIZone(.25);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        var robotPose = drivetrain.getState().Pose;
        var nearestTrench = trenches.get(0);

        for (Field.Trench t : trenches) {
            var distToNearest = getDistBetween(robotPose, nearestTrench.location);
            var distToT = getDistBetween(robotPose, t.location);

            if (distToT < distToNearest) {
                nearestTrench = t;
            }
        }

        this.entrance = nearestTrench.entrance;
        this.exit = nearestTrench.exit;

        if (nearestTrench.zone.contains(robotPose.getTranslation())) {
            this.cancel(); // NOOOO!!!!
        }

        if (getDistBetween(robotPose, exit) < getDistBetween(robotPose, entrance)) {
            this.entrance = nearestTrench.exit;
            this.exit = nearestTrench.entrance;
        }

        robotTargetDirection = entrance.getX() > robotPose.getX()
            ? Rotation2d.kZero
            : Rotation2d.k180deg;

        entrancePublisher.set(entrance);
        exitPublisher.set(exit);
        inTrenchYVelController.setSetpoint(this.entrance.getY());
    }

    @Override
    public void execute() {
        var robotPose = drivetrain.getState().Pose;
        var sourceDriveRequest = driveRequestSupplier.get();
        double driverX = sourceDriveRequest.VelocityX;

        var velocityY = inTrenchYVelController.calculate(robotPose.getY());
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            velocityY *= -1;
        }

        var correctedDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(10,0,0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(sourceDriveRequest.Deadband)
            .withTargetDirection(robotTargetDirection)
            .withVelocityX(driverX)
            .withVelocityY(velocityY);

        drivetrain.setControl(correctedDrive);
        yVelPublisher.set(velocityY);
    }

    private double getDistBetween(Pose2d pose1, Pose2d pose2) {
        return Math.abs(pose1.minus(pose2).getTranslation().getNorm());
    }
}
