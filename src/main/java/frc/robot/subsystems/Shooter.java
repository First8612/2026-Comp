package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
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
    private final List<BaseStatusSignal> signals = new ArrayList<BaseStatusSignal>();

    private Follower shootFollow = new Follower(20, MotorAlignmentValue.Opposed);

    Boolean isAiming = false;
    Optional<Double> aimingDistOverride = Optional.empty();
    private double flywheelSpeedGoal = 0;
    private boolean isFeeding = false;
    private boolean isFeedReversed = false;

    InterpolatingArrayTreeMap shootCalc = new InterpolatingArrayTreeMap(2);

    double currHoodGoal = 0; // number used w/ PID
    private final Debouncer flywheelReadyDebounce = new Debouncer(0.0, DebounceType.kRising);
    private TargetTracker targetTracker;
    private boolean overrideAim = false;
    private double hoodOverride = 0.0;
    private double flywheelOverride = 50.0;
    private boolean hasReset = false;
    private boolean warmingUp = false;
    private BooleanSupplier isClimbedSupplier;

    private PositionVoltage hoodPositionControl = new PositionVoltage(0).withSlot(0);
    private VelocityVoltage flywheelVelocityControl = new VelocityVoltage(0).withSlot(0);
    private VelocityVoltage feedVelocityControl = new VelocityVoltage(0).withSlot(0);
    private CoastOut coastControl = new CoastOut();


    public Shooter(TargetTracker targetTracker, BooleanSupplier isClimbedSupplier) {
        super();
        this.isClimbedSupplier = isClimbedSupplier;
        shootMotorLeftSignals.addTo(signals);
        shootMotorRightSignals.addTo(signals);
        hoodMotorSignals.addTo(signals);
        feedMotorSignals.addTo(signals);

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
        shootCalc.put(2.7, new double[] { 0.0, 69.0 });
        shootCalc.put(4.2, new double[] { 0.5, 80.0 });
        shootCalc.put(5.6, new double[] { 1.35, 93.0 });
        shootCalc.put(100.0, new double[] { 2.8, 100.0 });
        shootCalc.put(10000.0, new double[] { 2.8, 100.0 });

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
        .apply(new Slot0Configs().
                        withKV(0.135).
                        withKP(.1).
                        withKI(0).
                        withKD(0)
                );

        shootMotorLeft.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withPeakReverseDutyCycle(0));
        shootMotorLeft.getConfigurator().apply(new VoltageConfigs().withPeakReverseVoltage(0));

        shootMotorRight.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withPeakReverseDutyCycle(0));
        shootMotorRight.setControl(shootFollow);

        shootMotorLeft.getConfigurator().apply(
                new Slot0Configs()
                        .withKV(.125)
                        .withKP(.7)
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
        aimingDistOverride = Optional.empty();
    }

    public void enableAiming(double distOverride) {
        isAiming = true;
        aimingDistOverride = Optional.of(distOverride);
    }

    public void stop() {
        isAiming = false;
        isFeeding = false;
        isFeedReversed = false;
        aimingDistOverride = Optional.empty();
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

    public void setWarmup() {
        warmingUp = true;
    }

    public void stopWarmup() {
        warmingUp = false;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(signals);

        overrideAim = SmartDashboard.getBoolean("Shooter/override/active", overrideAim);
        flywheelOverride = SmartDashboard.getNumber("Shooter/override/flywheel", flywheelOverride);
        hoodOverride = SmartDashboard.getNumber("Shooter/override/hood", hoodOverride);
        currHoodGoal = 0;
        flywheelSpeedGoal = 0;

        if(warmingUp) {
            flywheelSpeedGoal = 90;
        }
        
        if (isAiming) {
            var distance = targetTracker.getRobotToTargetTranslation().getNorm();

            var aimingDistOverride = this.aimingDistOverride;

            if (isClimbedSupplier.getAsBoolean()) {
                aimingDistOverride = Optional.of(4.1);
            }

            if (aimingDistOverride.isPresent()) {
                distance = aimingDistOverride.get();
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
            hoodMotor.setControl(hoodPositionControl.withPosition(currHoodGoal));
        }

        if (flywheelSpeedGoal == 0) {
            shootMotorLeft.setControl(coastControl);
        } else {
            // Cruising
            shootMotorLeft.setControl(flywheelVelocityControl.withVelocity(flywheelSpeedGoal));
        }
        if(targetTracker.getRobotToTargetTranslation().getNorm() > 100 && isFeeding) {
            feedMotor.setControl(feedVelocityControl.withVelocity(30.0));
        }
        else if (isFeeding) {
            var speedGoal = isFeedReversed ? -10.0 : 30.0;
            feedMotor.setControl(feedVelocityControl.withVelocity(speedGoal));
        } else {
            feedMotor.setControl(coastControl);
        }
        
        NT.putNumber("Distance", targetTracker.getRobotToTargetTranslation().getNorm());
        NT.putBoolean("isAiming", isAiming);
        NT.putBoolean("isFeeding", isFeeding);
        NT.putBoolean("readyToShoot", readyToShoot());
        NT.putTalonFX("shootMotorLeft", shootMotorLeftSignals);
        NT.putTalonFX("shootMotorRight", shootMotorRightSignals);
        NT.putNumber("shootMotor/targetVelocity", flywheelSpeedGoal);
        NT.putBoolean("shootMotor/ready", readyToShoot());
        NT.putTalonFX("hood/motor", hoodMotorSignals);
        NT.putBoolean("hood/ready", hoodReady());
        NT.putTalonFX("feed/motor", feedMotorSignals);
    }
}