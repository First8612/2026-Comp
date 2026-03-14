package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;

public class TalonFXState {
    public StatusSignal<Angle> position;
    public StatusSignal<AngularVelocity> velocity;
    public StatusSignal<Voltage> motorVoltage;
    public StatusSignal<Current> supplyCurrent;
    public StatusSignal<Current> statorCurrent;
    public StatusSignal<Double> closedLoopReference;

    /**
     * Creates a new TalonFXState by capturing the current signals from a TalonFX motor.
     * The signals are not refreshed; they reflect the last known values.
     * 
     * @param motor the TalonFX motor to capture state from
     * @return a new TalonFXState object containing all motor signals
     */
    public static TalonFXState capture(TalonFX motor) {
        TalonFXState state = new TalonFXState();
        state.position = motor.getPosition();
        state.velocity = motor.getVelocity();
        state.motorVoltage = motor.getMotorVoltage();
        state.supplyCurrent = motor.getSupplyCurrent();
        state.statorCurrent = motor.getStatorCurrent();
        state.closedLoopReference = motor.getClosedLoopReference();
        return state;
    }

    /**
     * Refreshes all signals in multiple state objects in a batch operation.
     * Uses BaseStatusSignal.refreshAll() to perform all reads in parallel for maximum efficiency.
     * 
     * @param states the TalonFXState objects to refresh
     */
    public static void refreshAll(TalonFXState... states) {
        // Collect all signals from all states
        StatusSignal<?>[] allSignals = new StatusSignal<?>[states.length * 6];
        int index = 0;
        for (TalonFXState state : states) {
            allSignals[index++] = state.position;
            allSignals[index++] = state.velocity;
            allSignals[index++] = state.motorVoltage;
            allSignals[index++] = state.supplyCurrent;
            allSignals[index++] = state.statorCurrent;
            allSignals[index++] = state.closedLoopReference;
        }
        // Use BaseStatusSignal.refreshAll() to refresh all signals in parallel
        BaseStatusSignal.refreshAll(allSignals);
    }

    public StatusSignal<?>[] asArray() {
        return new StatusSignal<?>[] {
            position,
            velocity,
            motorVoltage,
            supplyCurrent,
            statorCurrent,
            closedLoopReference
        };
    }
}
