package digilib.cameraPower;

public interface CameraPower {

    CameraPowerState getState();

    void setControl(CameraPowerRequest request);

    void update();

    void updateState();

    void updateTelemetry();

}
