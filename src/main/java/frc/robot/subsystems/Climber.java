package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBuses;
import frc.robot.utils.NetworkTableGroup;
import frc.robot.utils.TalonFXState;

public class Climber extends SubsystemBase{
    private final NetworkTableGroup NT = new NetworkTableGroup("Climber", true);
    private final TalonFX climbMotorRight = new TalonFX(21, CANBuses.intake);
    private final TalonFX climbMotorLeft = new TalonFX(20, CANBuses.intake);
    private final TalonFXState climbMotorLeftState = TalonFXState.capture(climbMotorLeft);
    private final TalonFXState climbMotorRightState = TalonFXState.capture(climbMotorRight);


    private Intake intake;

    private double currClimbGoal = 0;
    private boolean hasReset = false;

    public Climber(Intake intake) {
        super();

        this.intake = intake;

        var slot0Config = new Slot0Configs().
                        withKP(1).
                        withKI(0).
                        withKD(0);

        var rmConfig = new MotorOutputConfigs();
        rmConfig.NeutralMode = NeutralModeValue.Brake;
        rmConfig.Inverted = InvertedValue.CounterClockwise_Positive;

        climbMotorRight.getConfigurator().apply(rmConfig);
        climbMotorRight.getConfigurator()
            .apply(slot0Config);

        var lmConfig = new MotorOutputConfigs();
        lmConfig.NeutralMode = NeutralModeValue.Brake;
        lmConfig.Inverted = InvertedValue.CounterClockwise_Positive;

        climbMotorLeft.getConfigurator().apply(lmConfig);
        climbMotorLeft.getConfigurator()
            .apply(slot0Config);

        changeMotorLimits(20);
    }

    public Command getClimberZeroCommand() {
        var command = Commands.sequence(
            Commands.runOnce(() -> {changeMotorLimits(20); currClimbGoal = 0; hasReset = false;}),

            Commands.parallel(
                getClimbMotorZeroCommand("left", climbMotorLeft),
                getClimbMotorZeroCommand("right", climbMotorRight)
            ),

            Commands.runOnce(() -> {hasReset = true;}));

        command.addRequirements(this);

        return command;
    }

    private Command getClimbMotorZeroCommand(String motorName, TalonFX climbMotor) {
        var ramp = new SlewRateLimiter(0.02);

        var command = Commands.sequence(
            Commands.runOnce(() -> {
                NT.putBoolean(motorName + "/reset", false);
            }),

            // drive backwards
            Commands.deadline(
                    Commands.waitUntil(() -> Math.abs(climbMotor.getStatorCurrent().getValueAsDouble()) > 20),
                    Commands.run(() -> {
                        var dutyCycle = ramp.calculate(-0.2);
                        climbMotor.setControl(new DutyCycleOut(dutyCycle));
                    })),

            // reset
            Commands.runOnce(() -> {
                climbMotor.setPosition(-0.2);
                climbMotor.setControl(new DutyCycleOut(0));
                climbMotor.getConfigurator().apply(new HardwareLimitSwitchConfigs()
                        .withForwardLimitAutosetPositionEnable(true)
                        .withForwardLimitAutosetPositionValue(1.18));
                System.out.println("Climber " + motorName + " reset finished");
                NT.putBoolean(motorName + "/reset", true);
            }));

        return command;
    }


    private void changeMotorLimits(double current) {
        climbMotorLeft.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(current).withStatorCurrentLimitEnable(true));
        climbMotorRight.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(current).withStatorCurrentLimitEnable(true));
    }

    public void changeHeight(double by) {
        currClimbGoal += by;
    }

    public void raiseClimb() {
        if(intake.isRetracted()) {
            changeMotorLimits(30);
            currClimbGoal = 70;
        }
    }

    public void lowerClimb() {
        changeMotorLimits(20);
        currClimbGoal = 0;
    }

    public void useClimb() {
        if(intake.isRetracted()) {
            changeMotorLimits(200);
            currClimbGoal = 33;
        }
    }

    public void manualClimb(double change) {
        changeMotorLimits(10);
        currClimbGoal += change;
    }

    public boolean isOut() {
        return currClimbGoal > 0;
    }

    public boolean getHasReset() {
        return hasReset;
    }

    public boolean isAtClimb() {
        var height = (climbMotorLeftState.position.getValueAsDouble() + climbMotorRightState.position.getValueAsDouble()) / 2.0;
        return MathUtil.isNear(33, height, 1);
    }

    @Override
    public void periodic() {
        if(hasReset) {
            climbMotorLeft.setControl(new PositionVoltage(0).withSlot(0).withPosition(currClimbGoal));
            climbMotorRight.setControl(new PositionVoltage(0).withSlot(0).withPosition(currClimbGoal));
        }

        climbMotorLeftState.refresh();
        climbMotorRightState.refresh();

        NT.putNumber("ClimbGoal", currClimbGoal);
        NT.putBoolean("climbed", isAtClimb());
        NT.putTalonFX("left", climbMotorLeftState);
        NT.putTalonFX("right", climbMotorRightState);
    }
}
