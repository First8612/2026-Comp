package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardHelper {
    public static void putTalonFX(String name, TalonFX motor) {
        SmartDashboard.putNumber(name + "/position", motor.getPosition(false).getValueAsDouble());
        SmartDashboard.putNumber(name + "/velocity", motor.getVelocity(false).getValueAsDouble());
        SmartDashboard.putNumber(name + "/motorVoltage", motor.getMotorVoltage(false).getValueAsDouble());
        SmartDashboard.putNumber(name + "/supplyCurrent", motor.getSupplyCurrent(false).getValueAsDouble());
        SmartDashboard.putNumber(name + "/statorCurrent", motor.getStatorCurrent(false).getValueAsDouble());
        SmartDashboard.putNumber(name + "/closedLoopReference", motor.getClosedLoopReference(false).getValueAsDouble());
    }

    public static void putCANCoder(String name, CANcoder encoder) {
        SmartDashboard.putNumber(name + "/position", encoder.getPosition(false).getValueAsDouble());
        SmartDashboard.putNumber(name + "/velocity", encoder.getVelocity(false).getValueAsDouble());
        SmartDashboard.putNumber(name + "/positionAbsolute", encoder.getAbsolutePosition(false).getValueAsDouble());
    }
}
