package frc.robot.utils;

import java.util.Collection;
import java.util.List;

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
        state.position = motor.getPosition(false);
        state.velocity = motor.getVelocity(false);
        state.motorVoltage = motor.getMotorVoltage(false);
        state.supplyCurrent = motor.getSupplyCurrent(false);
        state.statorCurrent = motor.getStatorCurrent(false);
        state.closedLoopReference = motor.getClosedLoopReference(false);
        return state;
    }

    /**
     * Adds all signals from this state to the provided collection.
     *
     * @param collection the collection to add signals to
     */
    public void addTo(Collection<StatusSignal<?>> collection) {
        collection.add(position);
        collection.add(velocity);
        collection.add(motorVoltage);
        collection.add(supplyCurrent);
        collection.add(statorCurrent);
        collection.add(closedLoopReference);
    }


    /**
     * Adds all signals from this state to the provided collection.
     *
     * @param collection the collection to add signals to
     */
    public void addTo(List<BaseStatusSignal> collection) {
        collection.add(position);
        collection.add(velocity);
        collection.add(motorVoltage);
        collection.add(supplyCurrent);
        collection.add(statorCurrent);
        collection.add(closedLoopReference);
    }

    /**
     * Returns all signals from this state as a list.
     *
     * @return a list containing all motor signals
     */
    public java.util.List<StatusSignal<?>> toList() {
        return java.util.List.of(position, velocity, motorVoltage, supplyCurrent, statorCurrent, closedLoopReference);
    }

}
