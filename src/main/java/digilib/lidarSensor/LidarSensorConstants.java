package digilib.lidarSensor;

import java.util.function.IntSupplier;

public record LidarSensorConstants(String name,
                                   IntSupplier simDistanceSupplier) {
}
