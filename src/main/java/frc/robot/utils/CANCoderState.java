package frc.robot.utils;

import java.util.Collection;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.*;

public class CANCoderState {
    public StatusSignal<Angle> position;
    public StatusSignal<AngularVelocity> velocity;
    public StatusSignal<Angle> absolutePosition;

    /**
     * Creates a new CANCoderState by capturing the current signals from a CANcoder.
     * The signals are not refreshed; they reflect the last known values.
     *
     * @param encoder the CANcoder to capture state from
     * @return a new CANCoderState object containing all encoder signals
     */
    public static CANCoderState capture(CANcoder encoder) {
        CANCoderState state = new CANCoderState();
        state.position = encoder.getPosition(false);
        state.velocity = encoder.getVelocity(false);
        state.absolutePosition = encoder.getAbsolutePosition(false);
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
        collection.add(absolutePosition);
    }

    public StatusSignal<?>[] asArray() {
        return new StatusSignal<?>[] {
            position,
            velocity,
            absolutePosition
        };
    }
}
