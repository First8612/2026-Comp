// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.GoToClimb;
import frc.robot.commands.DixieHornCommand;
import frc.robot.commands.DriveAndFaceTargetCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveTrenchRun;
import frc.robot.commands.ShootSequence;
import frc.robot.controls.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.PositionAccuracyEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.MatchTimer;
import frc.robot.subsystems.TargetTracker;
import frc.robot.utils.NetworkTableGroup;

public class RobotContainer {
    public final static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public final static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static boolean prescisionMode = false;
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final NetworkTableGroup robotNT = new NetworkTableGroup("Robot", true);

    private final Controls controls = new Controls();
    
    // subsystems
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();

    // private final LeadingTargetTracker targetTracker = new LeadingTargetTracker(drivetrain);
    private final TargetTracker targetTracker = new TargetTracker(drivetrain);

    private final Storage storage = new Storage();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber(intake);
    private final Shooter shooter = new Shooter(targetTracker, climber::isAtClimb);
    private final Vision vision = new Vision(drivetrain);
    private final PositionAccuracyEstimator positionAccuracyEstimator = new PositionAccuracyEstimator(drivetrain::getCachedState);
    private final LightStrip lights = new LightStrip(positionAccuracyEstimator::getEstimation);

    // commands
    private final DriveAndFaceTargetCommand driveAndFaceTarget = new DriveAndFaceTargetCommand(controls, drivetrain, targetTracker);
    private final ShootSequence shoot = new ShootSequence(shooter, storage, targetTracker, false, climber::isAtClimb);
    private final ShootSequence shootSimple = new ShootSequence(shooter, storage, targetTracker, true, climber::isAtClimb);
    private final MatchTimer matchTimer = new MatchTimer();

    SendableChooser<Command> autonChooser;

    public RobotContainer() {
        intake.getClimber(climber);
        drivetrain.setPositionAccuracyEstimator(positionAccuracyEstimator);
        NamedCommands.registerCommand("EnableAiming", Commands.runOnce(shooter::enableAiming));
        NamedCommands.registerCommand("ShootSequence", shoot);
        NamedCommands.registerCommand("StopShoot", Commands.runOnce(() -> shooter.stop()));
        NamedCommands.registerCommand("FaceTarget", driveAndFaceTarget);
        NamedCommands.registerCommand("ExtendIntake", Commands.runOnce(() -> intake.extend()));
        NamedCommands.registerCommand("RetractIntake", Commands.runOnce(() -> intake.retract()));
        NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> intake.in(), intake));
        NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> intake.stop(), intake));
        NamedCommands.registerCommand("ExtendClimb", Commands.run(() -> climber.raiseClimb()));
        NamedCommands.registerCommand("Climb", new InstantCommand(() -> climber.useClimb()));
        NamedCommands.registerCommand("StartWarmup", new InstantCommand(() -> shooter.setWarmup()));
        NamedCommands.registerCommand("EndWarmup", new InstantCommand(() -> shooter.stopWarmup()));
        
        new EventTrigger("ExtendIntake").onTrue(Commands.runOnce(() -> intake.extend()));
        new EventTrigger("StartIntake").onTrue(Commands.runOnce(() -> intake.in()));
        new EventTrigger("RetractIntake").onTrue(Commands.runOnce(() -> intake.retract()));
        new EventTrigger("StopIntake").onTrue(new InstantCommand(() -> intake.stop(), intake));
        new EventTrigger("ExtendClimb").onTrue(Commands.run(() -> climber.raiseClimb()));

        drivetrain.configureAutoBuilder();
        configureBindings();
        autonChooser = AutoBuilder.buildAutoChooser("Testing Auton");

        SmartDashboard.putData("Auto Path", autonChooser);
        // RobotModeTriggers.autonomous().onTrue(shooter.getZeroCommand());
        RobotModeTriggers.teleop().onTrue(shooter.getZeroCommand());

        RobotModeTriggers.autonomous().onTrue(climber.getClimberZeroCommand());
        // //Comment this line out if running autonomous
        // RobotModeTriggers.teleop().onTrue(climber.getClimberZeroCommand());

        //Un-comment this line if running autonomous
        // RobotModeTriggers.teleop().onTrue(climber.raiseClimb());

        RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> shooter.stopWarmup()));

        Field.writeOnceToNT();
    }

    public void startGameTimer() {
        matchTimer.start();
    }

    public void stopGameTimer() {
        matchTimer.stop();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            new DriveCommand(drivetrain, controls)
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        controls.prescisionMode().whileTrue(Commands.startEnd(() -> {prescisionMode = true;}, () -> {prescisionMode = false;}));
        controls.driveAndFaceTarget().whileTrue(driveAndFaceTarget);
        controls.xLock().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
        controls.shoot().whileTrue(shoot);
        controls.shootSimiple().whileTrue(shootSimple);
        controls.horn().whileTrue(new DixieHornCommand());
        controls.intakeExtend().onTrue(intake.runOnce(intake::extend));
        controls.intakeRetract().onTrue(intake.runOnce(intake::retract));
        controls.fieldReset().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            vision.reset();
        }));
        controls.goToClimb().whileTrue(new GoToClimb(drivetrain));
        controls.conveyIn().whileTrue(new RunCommand(() -> storage.conveyIn(), storage));
        controls.conveyOut().whileTrue(new RunCommand(() -> storage.conveyOut(), storage));
        controls.trenchRun().whileTrue(new DriveTrenchRun(drivetrain, controls::getDriveRequest));
        
        controls.intake().whileTrue(Commands.startEnd(() -> intake.in(), () -> intake.stop(), intake));

        controls.feedOut().whileTrue(Commands.startEnd(
            () -> shooter.feedReverse(true),
            () -> shooter.feedReverse(false)));

        controls.raiseClimb().onTrue(new InstantCommand(() -> {climber.raiseClimb(); /*SmartDashboard.putBoolean("Climber/Putting Up", true);*/}));
        controls.lowerClimb().onTrue(new InstantCommand(() -> {climber.lowerClimb(); /*SmartDashboard.putBoolean("Climber/Putting Down", true);*/}));
        controls.useClimb().onTrue(new InstantCommand(() -> {climber.useClimb();}));
        controls.manualReset().onTrue(climber.getClimberZeroCommand());
        // controls.manualClimb().whileTrue(new RunCommand(() -> climber.manualClimb(controls.getClimbManual()), climber));

        // climber.setDefaultCommand(Commands.run(() -> {
        //     var jog = controls.getTestJogValue();
        //     if (jog.isPresent()) {
        //         climber.changeHeight(jog.getAsDouble());
        //     }
        // }, climber));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controls.sysIdDrivetrainDynamicForward().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controls.sysIdDrivetrainDynamicReverse().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controls.sysIdDrivetrainQuasistaticForward().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controls.sysIdDrivetrainQuasistaticReverse().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        controls.startSignalLogger().onTrue(Commands.runOnce(() -> {
            SignalLogger.start();
            System.out.println("******  SIGNALLOGGER STARTED *********");
        }));
        controls.stopSignalLogger().onTrue(Commands.runOnce(SignalLogger::stop));

        controls.zeroHood().onTrue(shooter.getZeroCommand());

        drivetrain.registerTelemetry(logger::telemeterize);

        matchTimer.aboutToChangeTrigger.onTrue(Commands.runOnce(() -> {
            controls.setRumble(0.2);
        })).onFalse(Commands.runOnce(() -> {
            controls.setRumble(0);
        }));
        
        matchTimer.onChangeTrigger.onTrue(Commands.runOnce(() -> {
            controls.setRumble(1);
        })).onFalse(Commands.runOnce(() -> {
            controls.setRumble(0);
        }));

        matchTimer.gameFinishedTrigger.onTrue(Commands.runOnce(() -> {
            if (DriverStation.getMatchNumber() != 0) {
                vision.triggerRewindCapture();
            }
        }));
    }

    public Command getAutonomousCommand() {
        var auton = autonChooser.getSelected();
        auton.addRequirements(drivetrain);
        return auton;
    }

    public void robotPeriodic() {
        robotNT.putNumber("batteryVoltage", RobotController.getBatteryVoltage());
        robotNT.putNumber("batteryVoltage", RobotController.getBatteryVoltage());
        robotNT.putNumber("inputVoltage", RobotController.getInputVoltage());
        robotNT.putNumber("inputCurrent", RobotController.getInputCurrent());
        robotNT.putNumber("brownoutVoltage", RobotController.getBrownoutVoltage());
        robotNT.putNumber("cpuTemp", RobotController.getCPUTemp());
        robotNT.putNumber("pitch", drivetrain.getPigeon2().getPitch(true).getValueAsDouble());
        robotNT.putNumber("roll", drivetrain.getPigeon2().getPitch(true).getValueAsDouble());
        robotNT.putNumber("yaw", drivetrain.getPigeon2().getPitch(true).getValueAsDouble());
    }
}