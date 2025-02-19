package digilib.cameras;

public interface Camera {

    CameraState getState();

    void setControl(CameraRequest request);

    void setRobotPoseMode();

    void update();

    void updateState();

    void updateTelemetry();
}
