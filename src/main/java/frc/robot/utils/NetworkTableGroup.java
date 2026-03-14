package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTableGroup {
    private final String groupName;
    private final boolean enabled;

    public NetworkTableGroup(String name, Boolean enabled) {
        super();
        this.groupName = name;
        this.enabled = enabled;
    }

    public void putTalonFX(String name, TalonFX motor) {
         if (!enabled) return;
        
       putTalonFX(name, TalonFXState.capture(motor));
    }

    public void putTalonFX(String name, TalonFXState state) {
        putNumber(name + "/position", state.position.getValueAsDouble());
        putNumber(name + "/velocity", state.velocity.getValueAsDouble());
        putNumber(name + "/motorVoltage", state.motorVoltage.getValueAsDouble());
        putNumber(name + "/supplyCurrent", state.supplyCurrent.getValueAsDouble());
        putNumber(name + "/statorCurrent", state.statorCurrent.getValueAsDouble());
        putNumber(name + "/closedLoopReference", state.closedLoopReference.getValueAsDouble());
    }

    public void putCANCoder(String name, CANcoder state) {
        if (!enabled) return;
        
        putCANCoder(name, CANCoderState.capture(state));
    }

    public void putCANCoder(String name, CANCoderState state) {
        putNumber(name + "/position", state.position.getValueAsDouble());
        putNumber(name + "/velocity", state.velocity.getValueAsDouble());
        putNumber(name + "/positionAbsolute", state.absolutePosition.getValueAsDouble());
    }

    public void putNumber(String name, double value) {
        if (!enabled) return;
        
        SmartDashboard.putNumber(groupName + "/" + name, value);
    }
    
    public void putBoolean(String name, boolean value) {
        if (!enabled) return;
        
        SmartDashboard.putBoolean(groupName + "/" + name, value);
    }

    public void putString(String name, String value) {
        if (!enabled) return;
        
        SmartDashboard.putString(groupName + "/" + name, value);
    }

    public void putValue(String name, double value) {
        putNumber(name, value);
    }

    public void putValue(String name, boolean value) {
        putBoolean(name, value);
    }

    public void putValue(String name, String value) {
        putString(name, value);
    }

    public void putData(String name, Sendable value) {
        if (!enabled) return;
        
        SmartDashboard.putData(name, value);
    }

}
