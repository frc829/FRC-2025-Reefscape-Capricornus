package digilib.objectDetectors;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;

public class ObjectDetectorTelemetry {
    private final DoublePublisher timestamp;
    private final BooleanPublisher objectDetectedPublisher;
    private final DoublePublisher distancePublisher;

    public ObjectDetectorTelemetry(
            String name,
            Distance maxTrueDistance,
            Distance minTrueDistance) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        this.timestamp = table.getDoubleTopic("Timestamp").publish();
        DoublePublisher maxTrueDistancePublisher = table.getDoubleTopic("Max True Distance").publish();
        DoublePublisher minTrueDistancePublisher = table.getDoubleTopic("Min True Distance").publish();
        this.distancePublisher = table.getDoubleTopic("Distance").publish();
        this.objectDetectedPublisher = table.getBooleanTopic("Object Detected").publish();
        maxTrueDistancePublisher.set(maxTrueDistance.baseUnitMagnitude());
        minTrueDistancePublisher.set(minTrueDistance.baseUnitMagnitude());
    }

    public void telemeterize(ObjectDetectorState state){
        timestamp.set(state.getTimestamp().baseUnitMagnitude());
        objectDetectedPublisher.set(state.isInRange());
        distancePublisher.set(state.getDistance().magnitude());
    }

}
