package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveToLocation extends Command {
    private Drivetrain drivetrain;
    private Pose2d targetLocation;

    private final SwerveRequest.FieldCentric driveRequest = 
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    
    private PIDController xController = new PIDController(1, 0, 0);
    private PIDController yController = new PIDController(1, 0, 0);
    private PIDController oController = new PIDController(1.5, 0.1, 0);
    private Translation2d translationDiff = new Translation2d();
    private Rotation2d rotationDiff = new Rotation2d();
    private Debouncer atLocationDebounce = new Debouncer(0.06, DebounceType.kRising);
    private Timer timer = new Timer();
    private Supplier<Pose2d> locationSupplier;

    public DriveToLocation(Drivetrain drivetrain, Supplier<Pose2d> locationSupplier) {
        super();
        this.drivetrain = drivetrain;
        this.locationSupplier = locationSupplier;

        xController.setIZone(0.5);
        yController.setIZone(0.5);
        oController.setIZone(0.5);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        oController.reset();

        targetLocation = locationSupplier.get();

        timer.restart();
    }

    @Override
    public boolean isFinished() {

        return atLocationDebounce.calculate(
            translationDiff.getX() < 0.02
            && translationDiff.getY() < 0.02
            && rotationDiff.getRadians() < 0.01
        );
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drivetrain.setControl(driveRequest);
    }

    @Override
    public void execute() {
        var robotPose = drivetrain.getCachedState();

        translationDiff = robotPose.Pose.minus(targetLocation)
            .getTranslation();
        rotationDiff = robotPose.Pose.getRotation().minus(targetLocation.getRotation());
        

        var maxTranslation = 0.75;
        var maxRotation = 0.75;
        var ySpeed = 
            MathUtil.clamp(
                xController.calculate(translationDiff.getY()), 
                -maxTranslation, 
                maxTranslation) 
            * RobotContainer.MaxSpeed;
        var xSpeed = 
            MathUtil.clamp(
                yController.calculate(translationDiff.getX()), 
                -maxTranslation, 
                maxTranslation) 
            * RobotContainer.MaxSpeed;
        var oSpeed = 
            MathUtil.clamp(
                oController.calculate(rotationDiff.getRadians()), 
                -maxRotation, 
                maxRotation) 
            * RobotContainer.MaxAngularRate;

        
        drivetrain.setControl(
            driveRequest
                .withVelocityX(xSpeed)            
                .withVelocityY(ySpeed)            
                .withRotationalRate(oSpeed)            
        );

        SmartDashboard.putNumber("AutoTower/seconds", timer.get());
        SmartDashboard.putNumber("AutoTower/translation/x", translationDiff.getX());
        SmartDashboard.putNumber("AutoTower/translation/y", translationDiff.getY());
        SmartDashboard.putNumber("AutoTower/translation/o", rotationDiff.getRadians());
    }







}
