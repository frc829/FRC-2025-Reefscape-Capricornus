package digilib.claws;

import edu.wpi.first.units.measure.MutTime;

import static edu.wpi.first.units.Units.Seconds;

public class ClawState {

    private ClawValue clawValue = null;
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

    public ClawState withTimestamp(double seconds) {
        this.timestamp.mut_setBaseUnitMagnitude(seconds);
        return this;
    }
}
