package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardHelper {
    public static void putTalonFX(String name, TalonFX motor) {
        putTalonFX(name, TalonFXState.capture(motor));
    }

    public static void putTalonFX(String name, TalonFXState state) {
        SmartDashboard.putNumber(name + "/position", state.position.getValueAsDouble());
        SmartDashboard.putNumber(name + "/velocity", state.velocity.getValueAsDouble());
        SmartDashboard.putNumber(name + "/motorVoltage", state.motorVoltage.getValueAsDouble());
        SmartDashboard.putNumber(name + "/supplyCurrent", state.supplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber(name + "/statorCurrent", state.statorCurrent.getValueAsDouble());
        SmartDashboard.putNumber(name + "/closedLoopReference", state.closedLoopReference.getValueAsDouble());
    }

    public static void putCANCoder(String name, CANcoder encoder) {
        putCANCoder(name, CANCoderState.capture(encoder));
    }

    public static void putCANCoder(String name, CANCoderState state) {
        SmartDashboard.putNumber(name + "/position", state.position.getValueAsDouble());
        SmartDashboard.putNumber(name + "/velocity", state.velocity.getValueAsDouble());
        SmartDashboard.putNumber(name + "/positionAbsolute", state.absolutePosition.getValueAsDouble());
    }
}
