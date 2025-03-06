package digilib.lidarSensor;

public interface LidarSensor {

    LidarSensorState getState();

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState();
}
