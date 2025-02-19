package digilib.pneumatics;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.Seconds;

public class PneumaticsState {

    private Boolean compressorOn = null;
    private final MutTime timestamp = Seconds.mutable(0.0);

    public boolean isCompressorOn() {
        return compressorOn;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public PneumaticsState withCompressorOn(boolean compressorOn) {
        this.compressorOn = compressorOn;
        return this;
    }

    public PneumaticsState withTimestamp(double seconds) {
        this.timestamp.mut_setBaseUnitMagnitude(seconds);
        return this;
    }
}
