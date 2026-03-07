package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardHelper {
    public static void putTalonFX(String name, TalonFX motor) {
        SmartDashboard.putNumber(name + "/position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(name + "/velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(name + "/motorVoltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(name + "/supplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(name + "/statorCurrent", motor.getStatorCurrent().getValueAsDouble());
    }

    public static void putCANCoder(String name, CANcoder encoder) {
        SmartDashboard.putNumber(name + "/position", encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(name + "/velocity", encoder.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(name + "/positionAbsolute", encoder.getAbsolutePosition().getValueAsDouble());
    }
}
