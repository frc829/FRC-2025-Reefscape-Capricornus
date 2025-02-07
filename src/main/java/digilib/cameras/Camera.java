package digilib.cameras;

import static digilib.cameras.CameraState.*;

public interface Camera {

    public void setControl(CameraRequest request);

    public CameraState getState();

    public CameraState getStateCopy();

    public CameraState getLastState();

    public void updateTelemetry();

    public void setMode(CameraMode mode);

    public void update();







}
