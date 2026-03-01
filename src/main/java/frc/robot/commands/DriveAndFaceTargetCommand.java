package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Controls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.TargetTracker;

public class DriveAndFaceTargetCommand extends Command {
    private final Controls controls;
    private final Drivetrain drivetrain;
    private final TargetTracker targetTracker;

    private final SwerveRequest.FieldCentricFacingAngle driveFacingTarget = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(10,0,0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        
        
    public DriveAndFaceTargetCommand(Controls controls, Drivetrain drivetrain, TargetTracker targetTracker) {
        super();
        this.controls = controls;
        this.drivetrain = drivetrain;
        this.targetTracker = targetTracker;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        var sourceRequest = controls.getDriveRequest();

        var drive = driveFacingTarget
                .withDeadband(sourceRequest.Deadband)
                .withVelocityX(sourceRequest.VelocityX)
                .withVelocityY(sourceRequest.VelocityY)
                .withTargetDirection(targetTracker.getRobotToTargetRotation());

        drivetrain.setControl(drive);
    }
}
