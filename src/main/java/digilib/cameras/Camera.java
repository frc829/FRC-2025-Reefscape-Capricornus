package digilib.cameras;

public interface Camera {

    CameraState getState();

    void update();

    void updateState();

    void updateTelemetry();
}