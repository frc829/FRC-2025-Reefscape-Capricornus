package digilib.objectDetectors;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class ObjectDetectorState {

    private boolean inRange = false;
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final MutDistance distance = Meters.mutable(0.0);

    public boolean isInRange() {
        return inRange;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public MutDistance getDistance() {
        return distance;
    }

    public ObjectDetectorState withInRange(boolean inRange) {
        this.inRange = inRange;
        return this;
    }

    public ObjectDetectorState withDistance(double meters) {
        this.distance.mut_setBaseUnitMagnitude(meters);
        return this;
    }

    public ObjectDetectorState withTimestamp(double seconds) {
        this.timestamp.mut_replace(timestamp);
        return this;
    }
}
