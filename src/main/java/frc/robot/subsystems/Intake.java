package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.commands.DixieHornCommand;

public class Intake extends SubsystemBase {
    private final CANBus intakeCANBus = new CANBus("Intake");
    private final TalonFX intakeMotor = new TalonFX(10, intakeCANBus);
    private final TalonFX intakeExtendLeft = new TalonFX(11, intakeCANBus);
    private final Slot0Configs intakeExtendLeftSlot0Config;
    private final TalonFX intakeExtendRight = new TalonFX(12, intakeCANBus);
    private final CANcoder extendEncoder = new CANcoder(13, intakeCANBus);

    // these are off the absolute encoder. 
    // to use KG in feed-forward, the horizontal angle should be "0".
    private final Angle retractedGoal = Rotations.of(-0.199);
    private final Angle extendedGoal = Rotations.of(0);

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

        extendEncoder.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(Rotations.of(-0.198))
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)));

        // Extension Left (Leader)
        intakeExtendLeft.getConfigurator().apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(100)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(false)));
        intakeExtendLeft.setPosition(extendEncoder.getAbsolutePosition().getValueAsDouble() * extendRatio);
        intakeExtendRight.setPosition(extendEncoder.getAbsolutePosition().getValueAsDouble() * extendRatio);

        intakeExtendLeftSlot0Config = new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(0)
                .withKG(1)
                .withKI(0)
                .withKD(0);
        intakeExtendLeft.getConfigurator().apply(intakeExtendLeftSlot0Config);

        // Extension Left (Follower)
        intakeExtendRight.setControl(intakeFollow);
        intakeExtendRight.getConfigurator().apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(10)
                .withStatorCurrentLimitEnable(true));

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
        intakeExtendLeftSlot0Config.kG += value * 0.1;
        intakeExtendLeft.getConfigurator().apply(intakeExtendLeftSlot0Config);
    }

    public void extend() {
        intakeExtendLeft.setControl(
            new PositionVoltage(extendedGoal)
            .withSlot(0));
    }

    public void retract() {
        intakeExtendLeft.setControl(
            new PositionVoltage(retractedGoal)
            .withSlot(0));
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
            0.1);
    }

    @Override
    public void periodic() {
        intakeMotor.set(speed);

        SmartDashboard.putNumber("Intake/speed", speed);
        SmartDashboard.putBoolean("Intake/extended", isExtended());
        SmartDashboard.putBoolean("Intake/retracted", isRetracted());
        SmartDashboard.putNumber("Intake/ExtendGoal", intakeExtendLeft.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Intake/ExtendPos", intakeExtendLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/ExtendCurrent", intakeExtendLeft.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/ExtendVolts", intakeExtendLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/EncoderAbsolute", extendEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Encoder", extendEncoder.getPosition().getValueAsDouble());
    }
}
