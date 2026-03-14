package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import frc.robot.CANBuses;
import frc.robot.commands.DixieHornCommand;
import frc.robot.utils.InterpolatingArrayTreeMap;
import frc.robot.utils.NetworkTableGroup;
import frc.robot.utils.TalonFXState;
import frc.robot.utils.TargetTracker;

public class Shooter extends SubsystemBase {
    private final NetworkTableGroup NT = new NetworkTableGroup("Shooter", true);
    private final TalonFX shootMotorLeft = new TalonFX(20, CANBuses.shooter);
    private final TalonFX shootMotorRight = new TalonFX(21, CANBuses.shooter);
    private final TalonFX hoodMotor = new TalonFX(22, CANBuses.shooter);
    private final TalonFX feedMotor = new TalonFX(23, CANBuses.shooter);
    private final TalonFXState shootMotorLeftSignals = TalonFXState.capture(shootMotorLeft);
    private final TalonFXState shootMotorRightSignals = TalonFXState.capture(shootMotorRight);
    private final TalonFXState hoodMotorSignals = TalonFXState.capture(hoodMotor);
    private final TalonFXState feedMotorSignals = TalonFXState.capture(feedMotor);

    private Follower shootFollow = new Follower(20, MotorAlignmentValue.Opposed);

    Boolean isAiming = false;
    Optional<Double> aimingDistOveride = Optional.empty();
    private double flywheelSpeedGoal = 0;
    private boolean isFeeding = false;
    private boolean isFeedReversed = false;

    InterpolatingArrayTreeMap shootCalc = new InterpolatingArrayTreeMap(2);

    double currHoodGoal = 0; // number used w/ PID
    private final Debouncer flywheelReadyDebounce = new Debouncer(0.07, DebounceType.kRising);
    private TargetTracker targetTracker;
    private boolean overrideAim = false;
    private double hoodOverride = 0.0;
    private double flywheelOverride = 50.0;
    private boolean hasReset = false;


    public Shooter(TargetTracker targetTracker) {
        super();

        //OLD NUMBER
        // shootCalc.put(0.0, new double[] { 0.0, 49.0 * 1.5 });
        // shootCalc.put(1.2, new double[] { 0.0, 49.0 * 1.5});
        // shootCalc.put(2.7, new double[] { 0.1 * 4, 46.0 * 1.5});
        // shootCalc.put(4.2, new double[] { 0.2 * 4, 52.5 * 1.5});
        // shootCalc.put(5.6, new double[] { 0.3 * 4, 58.0 * 1.5});
        // shootCalc.put(200.0, new double[] { 0.3 * 4, 58.0 * 1.5});

        //NEW NUMBERS
        
        // shootCalc.put(0.0, new double[] { 0.0, 71.0 });
        // shootCalc.put(1.2, new double[] { 0.0, 71.0 });
        
        shootCalc.put(0.0, new double[] { 0.0, 65.0 });
        shootCalc.put(1.8, new double[] { 0.0, 65.0 });
        shootCalc.put(2.7, new double[] { 0.0, 68.0 });
        shootCalc.put(4.2, new double[] { 0.5, 78.0 });
        shootCalc.put(5.6, new double[] { 1.35, 91.0 });
        shootCalc.put(100.0, new double[] { 1.35, 91.0 });

        this.targetTracker = targetTracker;
        var mConfig = new MotorOutputConfigs();
        mConfig.NeutralMode = NeutralModeValue.Brake;
        mConfig.Inverted = InvertedValue.Clockwise_Positive;
        hoodMotor.getConfigurator().apply(mConfig);
        hoodMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true));
        hoodMotor.getConfigurator()
        .apply(new Slot0Configs().
                        withKP(10).
                        withKI(.1).
                        withKD(0)
                );

        feedMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(100).withStatorCurrentLimitEnable(false));
        feedMotor.getConfigurator()
        .apply(new Slot0Configs().
                        withKV(0.11).
                        withKP(1000000).
                        withKI(0).
                        withKD(0)
                );

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

        NT.putBoolean("override/active", overrideAim);
        NT.putNumber("override/hood", hoodOverride);
        NT.putNumber("override/flywheel", flywheelOverride);
        NT.putBoolean("hood/resetting", false);
        NT.putBoolean("hood/reset", false);
    }

    public void feedReverse(Boolean reverse) {
        isFeedReversed = reverse;
    }

    public void enableFeeding() {
        enableFeeding(true);
    }


    public void enableFeeding(boolean enabled) {
        isFeeding = enabled;
        isFeedReversed = false;
    }

    public void enableAiming() {
        isAiming = true;
        aimingDistOveride = Optional.empty();
    }

    public void enableAiming(double distOverride) {
        isAiming = true;
        aimingDistOveride = Optional.of(distOverride);
    }

    public void stop() {
        isAiming = false;
        isFeeding = false;
        isFeedReversed = false;
        aimingDistOveride = Optional.empty();
    }

    public boolean readyToShoot() {
        return flywheelReady() && hoodReady();
    }

    private Boolean flywheelReady() {
        if (flywheelSpeedGoal == 0)
            return false;

        return flywheelReadyDebounce.calculate(
                Math.abs(flywheelSpeedGoal - shootMotorLeftSignals.velocity.getValueAsDouble()) < 2.5);
    }

    private Boolean hoodReady() {
        return true;
    }

    public Command getZeroCommand() {
        var velocityDebounce = new Debouncer(1, DebounceType.kRising);
        velocityDebounce.calculate(false);

        var command = Commands.sequence(
                Commands.runOnce(() -> {
                    NT.putBoolean("hood/resetting", true);
                    NT.putBoolean("hood/reset", false);
                }),
                Commands.deadline(
                        Commands.waitUntil(() -> hoodMotor.getStatorCurrent().getValueAsDouble() > 5),
                        Commands.run(() -> hoodMotor.setControl(new DutyCycleOut(-.1)))),
                Commands.runOnce(() -> {
                    hoodMotor.setPosition(-.08);
                    hoodMotor.setControl(new DutyCycleOut(0));
                    hoodMotor.getConfigurator().apply(new HardwareLimitSwitchConfigs()
                            .withForwardLimitAutosetPositionEnable(true)
                            .withForwardLimitAutosetPositionValue(1.18));
                    System.out.println("reset finished");
                    NT.putBoolean("hood/resetting", false);
                    NT.putBoolean("hood/reset", true);
                }),
                Commands.runOnce(() -> {hasReset = true;}));

        command.addRequirements(this);

        return command;
    }

    @Override
    public void periodic() {
        List<BaseStatusSignal> signals = new ArrayList<>();
        shootMotorLeftSignals.addTo(signals);
        shootMotorRightSignals.addTo(signals);
        hoodMotorSignals.addTo(signals);
        feedMotorSignals.addTo(signals);
        BaseStatusSignal.refreshAll(signals);

        overrideAim = SmartDashboard.getBoolean("Shooter/override/active", overrideAim);
        flywheelOverride = SmartDashboard.getNumber("Shooter/override/flywheel", flywheelOverride);
        hoodOverride = SmartDashboard.getNumber("Shooter/override/hood", hoodOverride);
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
        }

        if(overrideAim && isAiming) {
            currHoodGoal = hoodOverride;
            flywheelSpeedGoal = flywheelOverride;
        }

        if(hasReset) {
            hoodMotor.setControl(new PositionVoltage(0).withSlot(0).withPosition(currHoodGoal));
        }

        if (flywheelSpeedGoal == 0) {
            shootMotorLeft.setControl(new CoastOut());
        } else {
            // Cruising
            shootMotorLeft.setControl(new VelocityVoltage(flywheelSpeedGoal).withSlot(0));
        }

        if ((flywheelReady() && isFeeding) || isFeedReversed) {
            var speedGoal = isFeedReversed ? -30 : 80;
            feedMotor.setControl(new VelocityVoltage(speedGoal).withSlot(0));
        } else {
            feedMotor.setControl(new CoastOut());
        }
        
        NT.putNumber("Distance", targetTracker.getRobotToTargetTranslation().getNorm());
        NT.putBoolean("isAiming", isAiming);
        NT.putBoolean("isFeeding", isFeeding);
        NT.putBoolean("readyToShoot", readyToShoot());
        NT.putTalonFX("shootMotorLeft", shootMotorLeft);
        NT.putTalonFX("shootMotorRight", shootMotorRight);
        NT.putNumber("shootMotor/targetVelocity", flywheelSpeedGoal);
        NT.putTalonFX("hood/motor", hoodMotor);
        NT.putBoolean("hood/ready", hoodReady());
        NT.putTalonFX("feed/motor", feedMotor);
    }
}