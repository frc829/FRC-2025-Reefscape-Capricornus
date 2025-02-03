package frc.robot.mechanisms.claws;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.Seconds;

public class ClawState implements Cloneable{

    public enum ClawValue {
        OPEN,
        CLOSED,
        UNKNOWN
    }

    private ClawValue clawValue = ClawValue.UNKNOWN;
    private final MutTime timestamp = Seconds.mutable(0.0);

    public ClawValue getClawValue() {
        return clawValue;
    }

    public MutTime getTimestamp() {
        return timestamp;
    }

    public ClawState withClawValue(ClawValue clawValue) {
        this.clawValue = clawValue;
        return this;
    }

    public ClawState withTimestamp(Time timestamp) {
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public ClawState withClawState(ClawState clawState) {
        this.clawValue = clawState.clawValue;
        this.timestamp.mut_replace(clawState.timestamp);
        return this;
    }

    @Override
    public ClawState clone() {
        try {
            ClawState toReturn =  (ClawState) super.clone();
            toReturn.clawValue = clawValue;
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
