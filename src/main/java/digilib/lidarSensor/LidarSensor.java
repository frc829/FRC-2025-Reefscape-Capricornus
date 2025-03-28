package digilib.lidarSensor;

public interface LidarSensor {

    LidarSensorState getState();

    void update();

    void updateSimState();
}
