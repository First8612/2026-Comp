package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DixieHornCommand;

public class Intake extends SubsystemBase {
    private final CANBus intakeCANBus = new CANBus("Intake");
    private final TalonFX intakeMotor = new TalonFX(10, intakeCANBus);
    private final TalonFX intakeExtendLeft = new TalonFX(11, intakeCANBus);
    private final TalonFX intakeExtendRight = new TalonFX(12, intakeCANBus);
    private final CANcoder extendEncoder = new CANcoder(13, intakeCANBus);
    private double extendGoal = 0;
    private double speed = 0;

    private double extendRatio = 16 / 3;

    private Follower intakeFollow = new Follower(11, MotorAlignmentValue.Opposed);

    public Intake() {
        super();
        
        setDefaultCommand(Commands.runOnce(this::stop, this));
        DixieHornCommand.enrollSubsystemMotors(this, intakeMotor);

        intakeExtendLeft.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        );
        intakeExtendLeft.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(10).withStatorCurrentLimitEnable(true));

        intakeExtendRight.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        );
        intakeExtendRight.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(10).withStatorCurrentLimitEnable(true));

        intakeExtendLeft.setPosition(extendEncoder.getAbsolutePosition().getValueAsDouble() * extendRatio);

        intakeExtendRight.setControl(intakeFollow);
        intakeExtendLeft.getConfigurator().apply(
            new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0)
        );

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

    public void extend() {
        extendGoal = -1;
    }

    public void retract() {
        extendGoal = 0;
    }

    public Boolean isExtended() {
        return Math.abs(intakeExtendLeft.getPosition().getValueAsDouble() - extendGoal) < 0.1 && extendGoal < -0.5;
    }

    public Boolean isRetracted() {
        return Math.abs(intakeExtendLeft.getPosition().getValueAsDouble() - extendGoal) < 0.1 && extendGoal > -0.5;
    }

    @Override
    public void periodic() {
        intakeMotor.set(speed);

        intakeExtendLeft.setControl(new PositionVoltage(0).withSlot(0).withPosition(extendGoal));

        SmartDashboard.putNumber("Intake/speed", speed);
        SmartDashboard.putBoolean("Intake/extended", isExtended());
        SmartDashboard.putNumber("Intake/ExtendGoal", extendGoal);
        SmartDashboard.putNumber("Intake/ExtendPos", intakeExtendLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Encoder", extendEncoder.getAbsolutePosition().getValueAsDouble());
    }
}
