package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBuses;
import frc.robot.commands.DixieHornCommand;
import frc.robot.utils.SmartDashboardHelper;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(10, CANBuses.intake);
    private final TalonFX intakeExtendLeft = new TalonFX(11, CANBuses.intake);
    private final TalonFX intakeExtendRight = new TalonFX(12, CANBuses.intake);
    private final CANcoder extendEncoderLeft = new CANcoder(13, CANBuses.intake);
    private final CANcoder extendEncoderRight = new CANcoder(14, CANBuses.intake);
    private final Slot0Configs intakeExtendSlot0Config;

    // these are off the absolute encoder. 
    // to use KG in feed-forward, the horizontal angle should be "0".
    private final Angle retractedGoal = Rotations.of(-0.199);
    private final Angle extendedGoal = Rotations.of(0);

    private boolean extended = false;

    private double speed = 0;
    private double extendRatio = 16 / 3;

    private Follower intakeFollow = new Follower(11, MotorAlignmentValue.Opposed);

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
                .withMagnetOffset(Rotations.of(-0.042))
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

        intakeExtendSlot0Config = new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(-1)
                .withKP(10)
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
            intakeExtendLeft.getPosition().getValueAsDouble(), 
            0.1);
    }

    public Boolean isRetracted() {
        return MathUtil.isNear(
            retractedGoal.magnitude(), 
            intakeExtendLeft.getPosition().getValueAsDouble(), 
            0.01);
    }

    private void intakeExtendSetControl(ControlRequest request) {
        intakeExtendLeft.setControl(request);
        intakeExtendRight.setControl(request);
    }

    @Override
    public void periodic() {
        intakeMotor.set(speed);
        if(extended && isExtended()) {
            intakeExtendLeft.setControl(new CoastOut());
        }
        if(!extended && isRetracted()) {
            intakeExtendLeft.setControl(new CoastOut());
        }

        SmartDashboard.putNumber("Intake/speed", speed);
        SmartDashboard.putBoolean("Intake/extended", isExtended());
        SmartDashboard.putBoolean("Intake/retracted", isRetracted());
        SmartDashboardHelper.putTalonFX("Intake/ExtendMotorLeft", intakeExtendLeft);
        SmartDashboardHelper.putTalonFX("Intake/ExtendMotorRight", intakeExtendRight);
        SmartDashboardHelper.putCANCoder("Intake/ExtendEncoderLeft", extendEncoderLeft);
        SmartDashboardHelper.putCANCoder("Intake/ExtendEncoderRight", extendEncoderRight);
        SmartDashboard.putNumber("Intake/ExtendMotors/slot0/kG", intakeExtendSlot0Config.kG);
        SmartDashboard.putNumber("Intake/ExtendMotors/slot0/kP", intakeExtendSlot0Config.kP);
        SmartDashboard.putNumber("Intake/ExtendMotors/targetPosition", intakeExtendLeft.getClosedLoopReference().getValueAsDouble());
    }
}
