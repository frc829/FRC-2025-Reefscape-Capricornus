package frc.robot.mechanisms.cameras;

public interface Camera {

    public CameraRobotPoseState getCameraRobotPoseState();

    public void update();

    public void updateTelemetry();






}
