package frc.robot.mechanisms.cameras;

import static frc.robot.mechanisms.cameras.CameraState.*;

public interface Camera {

    public void setControl(CameraRequest request);

    public CameraState getState();

    public CameraState getStateCopy();

    public CameraState getLastState();

    public void updateTelemetry();

    public void setMode(CameraMode mode);

    public void update();







}
