// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DixieHornCommand;
import frc.robot.commands.DriveAndFaceTargetCommand;
import frc.robot.commands.ShootSequence;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Storage;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.vision.VisionPipeline;
import frc.robot.utils.LeadingTargetTracker;
import frc.robot.utils.TargetTracker;

public class RobotContainer {
    private final EventLoop loop = new EventLoop();
    public final static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public final static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystickDrive = new CommandXboxController(0);
    private final CommandXboxController joystickOperate = new CommandXboxController(1);
    
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();

    // private final LeadingTargetTracker targetTracker = new LeadingTargetTracker(drivetrain);
    private final TargetTracker targetTracker = new TargetTracker(drivetrain);

    private final Storage storage = new Storage();
    // private final Shooter Shooter = new Shooter(targetTracker);
    private final Vision vision = new Vision(drivetrain);

    private final DriveAndFaceTargetCommand driveAndFaceTarget = new DriveAndFaceTargetCommand(joystickDrive, drivetrain, targetTracker);
    // private final ShootSequence shoot = new ShootSequence(Shooter, storage, targetTracker, joystickDrive, drivetrain, false);
    // private final ShootSequence shootSimple = new ShootSequence(Shooter, storage, targetTracker, joystickDrive, drivetrain, true);

    private Timer gameTime = new Timer();
    private final double[] gameEvents = {/*Start 1st Shift*/10, /*2nd Shift*/35, /*3st Shift*/60, /*4th Shift*/85, /*Start Endgame*/110, /*End of Game*/140};

    // Intake intake = new Intake();

    SendableChooser<Command> autonChooser;

    public RobotContainer() {
        // NamedCommands.registerCommand("ShootSequence", shootSimple);

        CameraServer.startAutomaticCapture();
        configureBindings();
        drivetrain.configureAutoBuilder();
        autonChooser = AutoBuilder.buildAutoChooser("RI3D Auto");

        SmartDashboard.putData("Auto Path", autonChooser);
        // RobotModeTriggers.autonomous().onTrue(Shooter.getZeroCommand());
        // RobotModeTriggers.teleop().onTrue(Shooter.getZeroCommand());

        Field.writeOnceToNT();
    }

    private boolean atGameScheduleTime(double sec, double threshold) {
        for(int i = 0; i < gameEvents.length; i++) {
            if(Math.abs(sec - gameEvents[i]) < threshold && sec > gameEvents[i]) {
                return true;
            }
        }
        return false;
    }

    public void startGameTimer() {
        gameTime.start();
    }
    public void stopGameTimer() {
        gameTime.reset();
        gameTime.stop();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystickDrive.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystickDrive.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystickDrive.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        Supplier<Double> getIntakeSpeed = () -> {
            var speed = joystickOperate.getRightTriggerAxis()-joystickOperate.getLeftTriggerAxis();
            // var speed = 0.0;
            return speed;
        };
        // joystickOperate.rightTrigger(0.1).or(joystickOperate.leftTrigger(0.1)).whileTrue(new RunCommand(() -> intake.setSpeedRaw(getIntakeSpeed.get()), intake));
        // joystickOperate.rightTrigger(0.1).and(joystickOperate.leftTrigger(0.1)).whileFalse(new RunCommand(() -> intake.stop(), intake));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystickDrive.rightBumper().whileTrue(driveAndFaceTarget);


        // joystickOperate.a().whileTrue(shoot);
        // joystickOperate.b().whileTrue(shootSimple);
        joystickOperate.rightBumper().whileTrue(new DixieHornCommand());


        // joystickOperate.y().whileTrue(new RunCommand(() -> Shooter.inFeed()));
        // joystickOperate.x().whileTrue(Commands.startEnd(Shooter::enableAiming, Shooter::stop, Shooter));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystickDrive.back().and(joystickDrive.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystickDrive.back().and(joystickDrive.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystickDrive.start().and(joystickDrive.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystickDrive.start().and(joystickDrive.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        // Reset the field-centric heading on left bumper press.
        joystickDrive.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        BooleanEvent changeEvent = new BooleanEvent(loop, () -> atGameScheduleTime(gameTime.get(), 0.5));
        changeEvent.rising().ifHigh(() -> {
            joystickDrive.setRumble(RumbleType.kBothRumble, 1);
            joystickOperate.setRumble(RumbleType.kBothRumble, 1);
        });
        
        changeEvent.falling().ifHigh(() -> {
            joystickDrive.setRumble(RumbleType.kBothRumble, 0);
            joystickOperate.setRumble(RumbleType.kBothRumble, 0);
        });
        
        BooleanEvent aboutToChange = new BooleanEvent(loop, () -> atGameScheduleTime(gameTime.get() + 3, 1));
        aboutToChange.rising().ifHigh(() -> {
            joystickDrive.setRumble(RumbleType.kBothRumble, 0.2);
            joystickOperate.setRumble(RumbleType.kBothRumble, 0.2);
        });

        aboutToChange.falling().ifHigh(() -> {
            joystickDrive.setRumble(RumbleType.kBothRumble, 0);
            joystickOperate.setRumble(RumbleType.kBothRumble, 0);
        });

    }

    public Command getAutonomousCommand() {
        var auton = autonChooser.getSelected();
        auton.addRequirements(drivetrain);
        return auton;
    }

    public void robotPeriodic() {
        Target.periodic(drivetrain.getState().Pose);
        targetTracker.periodic();
        vision.periodic();
        loop.poll();
    }
}