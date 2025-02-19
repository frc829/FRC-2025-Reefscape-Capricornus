package digilib.cameras;

public interface CameraRequest {

    void apply(Camera camera);

    class RobotPose implements CameraRequest {
        @Override
        public void apply(Camera camera) {
            camera.setRobotPoseMode();
        }
    }
}
