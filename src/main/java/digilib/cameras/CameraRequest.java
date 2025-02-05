package digilib.cameras;

public interface CameraRequest {

    public void apply(Camera camera);

    public class RobotPose implements CameraRequest {
        @Override
        public void apply(Camera camera) {
            camera.setMode(CameraState.CameraMode.ROBOT_POSE);
        }
    }

}
