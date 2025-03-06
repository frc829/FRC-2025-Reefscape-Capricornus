package digilib.lidarSensor;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LidarSensorTelemetry {
    private final DoublePublisher distancePublisher;

    public LidarSensorTelemetry(
            String name) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        this.distancePublisher = table
                .getDoubleTopic("Distance [mm]")
                .publish();
    }

    public void telemeterize(LidarSensorState state) {
        distancePublisher.set(state.getDistanceMeters() * 1000.0);
    }
}
