package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBuses;
import frc.robot.commands.DixieHornCommand;
import frc.robot.utils.CANCoderState;
import frc.robot.utils.SmartDashboardHelper;
import frc.robot.utils.TalonFXState;

import java.util.ArrayList;
import java.util.List;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(10, CANBuses.intake);
    private final TalonFX intakeExtendLeft = new TalonFX(11, CANBuses.intake);
    private final TalonFX intakeExtendRight = new TalonFX(12, CANBuses.intake);
    private final CANcoder extendEncoderLeft = new CANcoder(13, CANBuses.intake);
    private final CANcoder extendEncoderRight = new CANcoder(14, CANBuses.intake);
    private final Slot0Configs intakeExtendSlot0Config;

    // Status signals - captured as states for efficient refresh management
    private TalonFXState extendLeftState;
    private TalonFXState extendRightState;
    private CANCoderState extendEncoderLeftState;
    private CANCoderState extendEncoderRightState;

    // these are off the absolute encoder. 
    // to use KG in feed-forward, the horizontal angle should be "0".
    private final Angle retractedGoal = Rotations.of(-0.199);
    private final Angle extendedGoal = Rotations.of(0);

    private boolean extended = false;
    private double speed = 0;
    // extendRatio = 16 / 3;

    public Intake() {
        super();
        
        setDefaultCommand(Commands.runOnce(this::stop, this));
        DixieHornCommand.enrollSubsystemMotors(this, intakeMotor, intakeExtendLeft, intakeExtendRight);

        // Extension CANcoder
        /*
         * When using an absolute sensor, such as a CANcoder, the sensor offset must be 
         * configured such that a position of 0 represents the arm being held 
         * horizontally forward. From there, the RotorToSensor ratio must be 
         * configured to the ratio between the absolute sensor and the Talon FX rotor.
         */

        extendEncoderLeft.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(Rotations.of(-0.195))
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)));

        extendEncoderRight.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(Rotations.of(-0.4765))
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

        intakeExtendSlot0Config = new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(-1)
                .withKP(20)
                .withKI(0)
                .withKD(0);
        
        var intakeCurrentLimits = new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(150)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(false);

        // Extension Left
        intakeExtendLeft.getConfigurator().apply(
            new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(extendEncoderLeft.getDeviceID())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withSensorToMechanismRatio(1)
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(intakeCurrentLimits)
                .withSlot0(intakeExtendSlot0Config));

        // Extension Right
        intakeExtendRight.getConfigurator().apply(
            new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(extendEncoderRight.getDeviceID())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withSensorToMechanismRatio(1)
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(intakeCurrentLimits)
                .withSlot0(intakeExtendSlot0Config));

        // Initialize the TalonFXState objects by capturing current signals
        extendLeftState = TalonFXState.capture(intakeExtendLeft);
        extendRightState = TalonFXState.capture(intakeExtendRight);
        extendEncoderLeftState = CANCoderState.capture(extendEncoderLeft);
        extendEncoderRightState = CANCoderState.capture(extendEncoderRight);

        // Configure update frequencies for status signals
        // Position and velocity signals update at 100 Hz for periodic monitoring
        BaseStatusSignal.setUpdateFrequencyForAll(100,
            intakeExtendLeft.getPosition(false),
            intakeExtendRight.getPosition(false),
            intakeExtendLeft.getVelocity(false),
            intakeExtendRight.getVelocity(false));
        
        // Closed loop reference updates at 50 Hz as it's primarily for dashboard display
        intakeExtendLeft.getClosedLoopReference(false).setUpdateFrequency(50);

    }

    public void in() {
        speed = 0.75;
    }

    public void out() {
        speed = -0.75;
    }

    // intended for prototyping time
    public void setSpeedRaw(double intakeSpeed) {
        this.speed = intakeSpeed;
    }

    public void stop() {
        speed = 0;
    }

    public void increaseTestValue(double value) {
        intakeExtendSlot0Config.kP += value * 0.05;
        intakeExtendLeft.getConfigurator().apply(intakeExtendSlot0Config);
        intakeExtendRight.getConfigurator().apply(intakeExtendSlot0Config);
    }

    public void extend() {
        intakeExtendSetControl(
            new PositionVoltage(extendedGoal)
            .withSlot(0));
        extended = true;
    }

    public void retract() {
        intakeExtendSetControl(
            new PositionVoltage(retractedGoal)
            .withSlot(0));
        extended = false;
    }

    public Boolean isExtended() {
        return MathUtil.isNear(
            extendedGoal.magnitude(), 
            extendLeftState.position.getValueAsDouble(),
            0.1);
    }

    public Boolean isRetracted() {
        return MathUtil.isNear(
            retractedGoal.magnitude(), 
            extendLeftState.position.getValueAsDouble(),
            0.01);
    }

    private void intakeExtendSetControl(ControlRequest request) {
        intakeExtendLeft.setControl(request);
        intakeExtendRight.setControl(request);
    }

    @Override
    public void periodic() {
        // Batch refresh all status signals in parallel using BaseStatusSignal.refreshAll()
        List<StatusSignal<?>> signals = new ArrayList<>();
        extendLeftState.addToArray(signals);
        extendRightState.addToArray(signals);
        extendEncoderLeftState.addToArray(signals);
        extendEncoderRightState.addToArray(signals);
        BaseStatusSignal.refreshAll(signals.toArray(new StatusSignal<?>[0]));

        if(isExtended()) {
            intakeMotor.set(speed);
        }else{
            intakeMotor.set(0);
        }
        if(extended && isExtended()) {
            intakeExtendSetControl(new NeutralOut());
        }
        if(!extended && isRetracted()) {
            intakeExtendSetControl(new NeutralOut());
        }

        SmartDashboard.putNumber("Intake/speed", speed);
        // SmartDashboard.putBoolean("Intake/extended", isExtended());
        // SmartDashboard.putBoolean("Intake/retracted", isRetracted());
        // SmartDashboardHelper.putTalonFX("Intake/ExtendMotorLeft", extendLeftState);
        // SmartDashboardHelper.putTalonFX("Intake/ExtendMotorRight", extendRightState);
        // SmartDashboardHelper.putCANCoder("Intake/ExtendEncoderLeft", extendEncoderLeftState);
        // SmartDashboardHelper.putCANCoder("Intake/ExtendEncoderRight", extendEncoderRightState);
    }
}
