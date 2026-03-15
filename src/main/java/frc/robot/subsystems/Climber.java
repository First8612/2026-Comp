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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBuses;
import frc.robot.utils.NetworkTableGroup;
import frc.robot.utils.TalonFXState;

public class Climber extends SubsystemBase{
    private final NetworkTableGroup NT = new NetworkTableGroup("Climber", false);
    private final TalonFX climbMotorRight = new TalonFX(21, CANBuses.intake);
    private final TalonFX climbMotorLeft = new TalonFX(20, CANBuses.intake);

    private double currClimbGoal = 0;
    private boolean hasReset = false;

    public Climber() {
        super();

        var slot0Config = new Slot0Configs().
                        withKP(12).
                        withKI(.1).
                        withKD(0);

        var rmConfig = new MotorOutputConfigs();
        rmConfig.NeutralMode = NeutralModeValue.Brake;
        rmConfig.Inverted = InvertedValue.CounterClockwise_Positive;

        climbMotorRight.getConfigurator().apply(rmConfig);
        climbMotorRight.getConfigurator()
            .apply(slot0Config);

        var lmConfig = new MotorOutputConfigs();
        lmConfig.NeutralMode = NeutralModeValue.Brake;
        lmConfig.Inverted = InvertedValue.Clockwise_Positive;

        climbMotorLeft.getConfigurator().apply(lmConfig);
        climbMotorLeft.getConfigurator()
            .apply(slot0Config);

        changeMotorLimits(20);

        currClimbGoal = climbMotorRight.getPosition().getValueAsDouble();
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
            // drive backwards
            Commands.deadline(
                    Commands.waitUntil(() -> Math.abs(climbMotor.getStatorCurrent().getValueAsDouble()) > 10),
                    Commands.run(() -> {
                        var dutyCycle = ramp.calculate(-0.1);
                        climbMotor.setControl(new DutyCycleOut(dutyCycle));
                    })),

            // reset
            Commands.runOnce(() -> {
                climbMotor.setPosition(-0.1);
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
        changeMotorLimits(10);
        currClimbGoal = 25;
    }

    public void lowerClimb() {
        changeMotorLimits(10);
        currClimbGoal = 0;
    }

    public void useClimb() {
        changeMotorLimits(150);
        currClimbGoal = 12;
    }

    public void manualClimb(double change) {
        changeMotorLimits(10);
        currClimbGoal += change;
    }

    @Override
    public void periodic() {
        if(hasReset) {
            climbMotorLeft.setControl(new PositionVoltage(0).withSlot(0).withPosition(currClimbGoal));
            climbMotorRight.setControl(new PositionVoltage(0).withSlot(0).withPosition(currClimbGoal));
        }

        NT.putNumber("ClimbGoal", currClimbGoal);
        NT.putTalonFX("left", climbMotorLeft);
        NT.putTalonFX("right", climbMotorRight);
    }
}
