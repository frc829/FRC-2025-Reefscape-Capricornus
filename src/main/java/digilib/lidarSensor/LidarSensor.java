package digilib.lidarSensor;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class LidarSensor {

    public record Config(String name) {
    }

    private final DoublePublisher distancePublisher;

    public LidarSensor(String name) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        distancePublisher = table
                .getDoubleTopic("Distance [mm]")
                .publish();
    }

    public abstract double getDistanceMillimeters();

    public void update() {
        distancePublisher.set(getDistanceMillimeters());
    }
}
