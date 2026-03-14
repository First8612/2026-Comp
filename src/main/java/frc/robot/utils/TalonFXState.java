package frc.robot.utils;

import java.util.Collection;

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
     * Adds all signals from this state to the provided collection.
     *
     * @param collection the collection to add signals to
     */
    public void addToArray(Collection<StatusSignal<?>> collection) {
        collection.add(position);
        collection.add(velocity);
        collection.add(motorVoltage);
        collection.add(supplyCurrent);
        collection.add(statorCurrent);
        collection.add(closedLoopReference);
    }
}
