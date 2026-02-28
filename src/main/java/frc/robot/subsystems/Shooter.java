package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DixieHornCommand;
import frc.robot.utils.InterpolatingArrayTreeMap;
import frc.robot.utils.TargetTracker;

public class Shooter extends SubsystemBase {
    // TalonFX shootMotor = new TalonFX(41);
    // TalonFX hoodMotor = new TalonFX(42);
    // TalonFX feedMotor = new TalonFX(43);
    // For actual robot when we switch over
    CANBus shooterCANBus = new CANBus("Shooter");
    TalonFX shootMotorLeft = new TalonFX(20, shooterCANBus);
    TalonFX shootMotorRight = new TalonFX(21, shooterCANBus);
    TalonFX hoodMotor = new TalonFX(22, shooterCANBus);
    TalonFX feedMotor = new TalonFX(23, shooterCANBus);

    private Follower shootFollow = new Follower(20, MotorAlignmentValue.Opposed);

    Boolean isAiming = false;
    Optional<Double> aimingDistOveride = Optional.empty();
    double feedDutyCycle = 0;
    double flywheelSpeedGoal = 0;
    InterpolatingArrayTreeMap shootCalc = new InterpolatingArrayTreeMap(2);

    double currHoodGoal = 0; // number used w/ PID
    private final Debouncer flywheelReadyDebounce = new Debouncer(0.07, DebounceType.kRising);
    private TargetTracker targetTracker;

    public Shooter(TargetTracker targetTracker) {
        super();

        shootCalc.put(0.0, new double[] { 0.0, 49.0 * 1.5 });
        shootCalc.put(1.2, new double[] { 0.0, 49.0 * 1.5});
        shootCalc.put(2.7, new double[] { 0.1 * 4, 46.0 * 1.5});
        shootCalc.put(4.2, new double[] { 0.2 * 4, 52.5 * 1.5});
        shootCalc.put(5.6, new double[] { 0.3 * 4, 58.0 * 1.5});
        shootCalc.put(200.0, new double[] { 0.3 * 4, 58.0 * 1.5});

        this.targetTracker = targetTracker;
        var mConfig = new MotorOutputConfigs();
        mConfig.NeutralMode = NeutralModeValue.Brake;
        mConfig.Inverted = InvertedValue.Clockwise_Positive;
        hoodMotor.getConfigurator().apply(mConfig);
        hoodMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true));
        
        feedMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(80).withStatorCurrentLimitEnable(false));

        var shooterCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true);

        shootMotorLeft.getConfigurator().apply(shooterCurrentLimits);
        shootMotorLeft.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withPeakReverseDutyCycle(0));
        shootMotorRight.getConfigurator().apply(shooterCurrentLimits);
        shootMotorRight.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withPeakReverseDutyCycle(0));
        shootMotorRight.setControl(shootFollow);

        /*
         * NOTES: (with two brass flywheels)
         * At KV of .13 it starts to overrun a little bit.
         * Adding KP caused recovery to be faster, but at 1 or above (with 0 I and D)
         * it would oscillate and not stay in the "ready" range, and you could hear
         * the belt vibrating.
         * At KV .125, KP .8 recovery time is about .5sec
         */
        shootMotorLeft.getConfigurator().apply(
                new Slot0Configs()
                        // TODO: still iterating on what these values should be
                        .withKV(.125) // this seems right
                        .withKP(0.7)
                        .withKI(0)
                        .withKD(0)

        );

        setDefaultCommand(Commands.runOnce(this::stop, this));
        DixieHornCommand.enrollSubsystemMotors(this, shootMotorLeft, shootMotorRight, feedMotor);

        SmartDashboard.putData("Shooter/system", this);
    }

    public void warmup() {
    }

    public void inFeed() {
        feedDutyCycle = 0.5;
    }

    public void backFeed() {
        feedDutyCycle = -0.25;
    }

    public void stopFeed() {
        feedDutyCycle = 0;
    }

    public void stop() {
        feedDutyCycle = 0;
        isAiming = false;
        aimingDistOveride = Optional.empty();
    }

    public void enableAiming() {
        isAiming = true;
        aimingDistOveride = Optional.empty();
    }

    public void enableAiming(double distOverride) {
        isAiming = true;
        aimingDistOveride = Optional.of(distOverride);
    }

    public boolean readyToShoot() {
        return flywheelReady() && hoodReady();
    }

    private Boolean flywheelReady() {
        if (flywheelSpeedGoal == 0)
            return false;

        return flywheelReadyDebounce.calculate(
                Math.abs(flywheelSpeedGoal - shootMotorLeft.getVelocity().getValueAsDouble()) < 2.5);
    }

    private Boolean hoodReady() {
        return true;
    }

    public void spinUp(double speed) {
        flywheelSpeedGoal = speed;
    }

    public Command getZeroCommand() {
        var command = Commands.sequence(
                Commands.deadline(
                        Commands.waitUntil(() -> Math.abs(hoodMotor.getStatorCurrent().getValueAsDouble()) > 16),
                        Commands.run(() -> hoodMotor.setControl(new DutyCycleOut(-0.1)))),
                Commands.runOnce(() -> {
                    hoodMotor.setPosition(-0.01);
                    hoodMotor.setControl(new DutyCycleOut(0));
                    hoodMotor.getConfigurator().apply(new HardwareLimitSwitchConfigs()
                            .withForwardLimitAutosetPositionEnable(true)
                            .withForwardLimitAutosetPositionValue(1.18));
                    System.out.println("reset finished");
                }));

        command.addRequirements(this);

        return command;
    }

    @Override
    public void periodic() {
        currHoodGoal = 0;
        flywheelSpeedGoal = 0;
        if (isAiming) {
            var distance = targetTracker.getRobotToTargetTranslation().getNorm();

            if (aimingDistOveride.isPresent()) {
                distance = aimingDistOveride.get();
            }

            double[] setAmounts = shootCalc.get(distance);

            currHoodGoal = setAmounts[0];
            flywheelSpeedGoal = setAmounts[1];
            SmartDashboard.putNumber("Shooter/speedGoal", flywheelSpeedGoal);
        }

        hoodMotor.setControl(new PositionVoltage(0).withSlot(0).withPosition(currHoodGoal));
        if (flywheelSpeedGoal == 0) {
            shootMotorLeft.setControl(new CoastOut());
        } else {
            // Cruising
            shootMotorLeft.setControl(new VelocityVoltage(flywheelSpeedGoal).withSlot(0));
        }

        if (flywheelReady()) {
            feedMotor.set(feedDutyCycle);
        } else {
            feedMotor.set(0);
        }
        
        SmartDashboard.putNumber("Shooter/Distance", targetTracker.getRobotToTargetTranslation().getNorm());
        SmartDashboard.putNumber("Shooter/shootMotor/voltageLeft", shootMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/shootMotor/voltageRight", shootMotorRight.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/shootMotor/velocity", shootMotorLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/shootMotor/currentLeft", shootMotorLeft.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/shootMotor/currentRight", shootMotorRight.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/shootMotor/value", shootMotorLeft.get());
        SmartDashboard.putBoolean("Shooter/isAiming", isAiming);
        SmartDashboard.putNumber("Shooter/hood/actual", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/hood/ready", hoodReady());
        SmartDashboard.putNumber("Shooter/hood/current", hoodMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/flywheelReady", flywheelReady());
        SmartDashboard.putNumber("Shooter/flywheelSpeedGoal", flywheelSpeedGoal);
        SmartDashboard.putBoolean("Shooter/readyToShoot", readyToShoot());
        SmartDashboard.putNumber("Shooter/feedSet", feedMotor.get());
        SmartDashboard.putNumber("Shooter/feedDutyCycleTarget", feedDutyCycle);
        SmartDashboard.putNumber("Shooter/hood/hoodGoal", currHoodGoal);

    }
}
