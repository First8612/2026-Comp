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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANBuses;
import frc.robot.utils.SmartDashboardHelper;

public class Climber extends SubsystemBase{
    private final TalonFX climbMotorRight = new TalonFX(21, CANBuses.intake);
    private final TalonFX climbMotorLeft = new TalonFX(20, CANBuses.intake);
    private double currClimbGoal = 0;
    private boolean hasReset = false;

    public Climber() {
        super();

        var rmConfig = new MotorOutputConfigs();
        rmConfig.NeutralMode = NeutralModeValue.Brake;
        rmConfig.Inverted = InvertedValue.CounterClockwise_Positive;

        climbMotorLeft.getConfigurator().apply(rmConfig);
        climbMotorLeft.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true));
        climbMotorLeft.getConfigurator()
        .apply(new Slot0Configs().
                        withKP(8).
                        withKI(.1).
                        withKD(0)
                );

        var lmConfig = new MotorOutputConfigs();
        lmConfig.NeutralMode = NeutralModeValue.Brake;
        lmConfig.Inverted = InvertedValue.Clockwise_Positive;

        climbMotorLeft.getConfigurator().apply(lmConfig);
        climbMotorLeft.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true));
        climbMotorLeft.getConfigurator()
        .apply(new Slot0Configs().
                        withKP(8).
                        withKI(.1).
                        withKD(0)
                );
    }

    public Command getClimberZeroCommand() {
        var command = Commands.sequence(
            Commands.runOnce(() -> {changeMotorLimits(20); currClimbGoal = 0; hasReset = false;}),
            Commands.parallel(
                Commands.sequence(
                    Commands.deadline(
                            Commands.waitUntil(() -> Math.abs(climbMotorLeft.getStatorCurrent().getValueAsDouble()) > 15),
                            Commands.run(() -> climbMotorLeft.setControl(new DutyCycleOut(-0.1)))),
                    Commands.runOnce(() -> {
                        climbMotorLeft.setPosition(-0.1);
                        climbMotorLeft.setControl(new DutyCycleOut(0));
                        climbMotorLeft.getConfigurator().apply(new HardwareLimitSwitchConfigs()
                                .withForwardLimitAutosetPositionEnable(true)
                                .withForwardLimitAutosetPositionValue(1.18));
                        System.out.println("reset finished");
                        SmartDashboard.putBoolean("Climber/resetLeft", true);
                    })),
                Commands.sequence(
                    Commands.deadline(
                            Commands.waitUntil(() -> Math.abs(climbMotorRight.getStatorCurrent().getValueAsDouble()) > 15),
                            Commands.run(() -> climbMotorRight.setControl(new DutyCycleOut(-0.1)))),
                    Commands.runOnce(() -> {
                        climbMotorRight.setPosition(-0.1);
                        climbMotorRight.setControl(new DutyCycleOut(0));
                        climbMotorRight.getConfigurator().apply(new HardwareLimitSwitchConfigs()
                                .withForwardLimitAutosetPositionEnable(true)
                                .withForwardLimitAutosetPositionValue(1.18));
                        System.out.println("reset finished");
                        SmartDashboard.putBoolean("Climber/resetLeft", true);
                    }))),
            Commands.runOnce(() -> {hasReset = true;}));

        command.addRequirements(this);

        return command;
    }
    private void changeMotorLimits(double current) {
        climbMotorLeft.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(current).withStatorCurrentLimitEnable(true));
        climbMotorRight.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(current).withStatorCurrentLimitEnable(true));
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
        changeMotorLimits(50);
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

        // SmartDashboard.putNumber("Climber/ClimbGoal", currClimbGoal);
        // SmartDashboardHelper.putTalonFX("Climber/LeftMotor", climbMotorLeft);
        // SmartDashboardHelper.putTalonFX("Climber/RightMotor", climbMotorRight);
    }
}
