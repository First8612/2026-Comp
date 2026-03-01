// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DixieHornCommand;
import frc.robot.commands.DriveAndFaceTargetCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SafeRobotForTrench;
import frc.robot.commands.ShootSequence;
import frc.robot.controls.Controls;
import frc.robot.controls.TestingControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Storage;
import edu.wpi.first.cameraserver.*;
import frc.robot.utils.TargetTracker;

public class RobotContainer {
    public final static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public final static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final EventLoop loop = new EventLoop();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Controls controls = new TestingControls();
    
    // subsystems
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();

    // private final LeadingTargetTracker targetTracker = new LeadingTargetTracker(drivetrain);
    private final TargetTracker targetTracker = new TargetTracker(drivetrain);

    private final Storage storage = new Storage();
    private final Shooter shooter = new Shooter(targetTracker);
    private final Vision vision = new Vision(drivetrain);

    // commands
    private final DriveAndFaceTargetCommand driveAndFaceTarget = new DriveAndFaceTargetCommand(controls, drivetrain, targetTracker);
    private final ShootSequence shoot = new ShootSequence(shooter, storage, targetTracker, drivetrain, false);
    private final ShootSequence shootSimple = new ShootSequence(shooter, storage, targetTracker, drivetrain, true);
    private Timer gameTime = new Timer();
    private final double[] gameEvents = {/*Start 1st Shift*/10, /*2nd Shift*/35, /*3st Shift*/60, /*4th Shift*/85, /*Start Endgame*/110, /*End of Game*/140};

    private final Intake intake = new Intake();

    // events
    private final BooleanEvent inTrenchEvent = new BooleanEvent(loop, () -> {
        var robotPose = drivetrain.getState().Pose;
        return Field.inTrenchZone(robotPose);
    });

    SendableChooser<Command> autonChooser;

    public RobotContainer() {
        // NamedCommands.registerCommand("ShootSequence", shootSimple);

        CameraServer.startAutomaticCapture();
        configureBindings();
        drivetrain.configureAutoBuilder();
        autonChooser = AutoBuilder.buildAutoChooser("RI3D Auto");

        SmartDashboard.putData("Auto Path", autonChooser);
        RobotModeTriggers.autonomous().onTrue(shooter.getZeroCommand());
        RobotModeTriggers.teleop().onTrue(shooter.getZeroCommand());

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
            new DriveCommand(drivetrain, controls)
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        controls.driveAndFaceTarget().whileTrue(driveAndFaceTarget);
        controls.shoot().whileTrue(shoot);
        controls.shootSimiple().whileTrue(shootSimple);
        controls.horn().whileTrue(new DixieHornCommand());
        controls.intakeExtend().onTrue(intake.runOnce(intake::extend));
        controls.intakeRetract().onTrue(intake.runOnce(intake::retract));
        controls.fieldReset().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controls.sysIdDrivetrainDynamicForward().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controls.sysIdDrivetrainDynamicReverse().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controls.sysIdDrivetrainQuasistaticForward().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controls.sysIdDrivetrainQuasistaticReverse().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);

         // Field events
         var safeRobotForTrench = new SafeRobotForTrench(intake, shooter);
        SmartDashboard.putBoolean("Field/inTrench", false);
         inTrenchEvent.rising().ifHigh(() -> {
            SmartDashboard.putBoolean("Field/inTrench", true);
            CommandScheduler.getInstance().schedule(safeRobotForTrench);
         });
         inTrenchEvent.falling().ifHigh(() -> {
            SmartDashboard.putBoolean("Field/inTrench", false);
         });
        
        BooleanEvent changeEvent = new BooleanEvent(loop, () -> atGameScheduleTime(gameTime.get(), 0.5));
        changeEvent.rising().ifHigh(() -> {
            controls.setRumble(1);
            controls.setRumble(1);
        });
        
        changeEvent.falling().ifHigh(() -> {
            controls.setRumble(0);
            controls.setRumble(0);
        });
        
        BooleanEvent aboutToChange = new BooleanEvent(loop, () -> atGameScheduleTime(gameTime.get() + 3, 1));
        aboutToChange.rising().ifHigh(() -> {
            controls.setRumble(0.2);
            controls.setRumble(0.2);
        });

        aboutToChange.falling().ifHigh(() -> {
            controls.setRumble(0);
            controls.setRumble(0);
        });
    }

    public Command getAutonomousCommand() {
        var auton = autonChooser.getSelected();
        auton.addRequirements(drivetrain);
        return auton;
    }

    public void robotPeriodic() {
        loop.poll();
        Target.periodic(drivetrain.getState().Pose);
        targetTracker.periodic();
        vision.periodic();
    }
}