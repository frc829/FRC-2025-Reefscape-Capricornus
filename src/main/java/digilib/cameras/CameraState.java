package digilib.cameras;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Map;

public class CameraState {


    public enum CameraMode {
        ROBOT_POSE(0);

        private final int pipelineIndex;

        CameraMode(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        private static final Map<Integer, CameraMode> map = Map.of(0, ROBOT_POSE);

        public int getPipelineIndex() {
            return pipelineIndex;
        }

        public static CameraMode get(int pipelineIndex) {
            return map.get(pipelineIndex);
        }
    }

    private String cameraMode = "N/A";
    private Pose3d robotPose = new Pose3d(Double.NaN, Double.NaN, Double.NaN, new Rotation3d(Double.NaN, Double.NaN, Double.NaN));
    private Matrix<N3, N1> robotPoseStdDev = MatBuilder
            .fill(Nat.N3(),
                    Nat.N1(),
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE);
    private double timestampSeconds = 0.0;

    public String getCameraMode() {
        return cameraMode;
    }

    public void setCameraMode(CameraMode cameraMode) {
        this.cameraMode = cameraMode.name();
    }

    public Pose3d getRobotPose() {
        return robotPose;
    }

    public void setRobotPose(Pose3d robotPose) {
        this.robotPose = robotPose;
    }

    public Matrix<N3, N1> getRobotPoseStdDev() {
        return robotPoseStdDev;
    }

    public void setRobotPoseStdDev(Matrix<N3, N1> robotPoseStdDev) {
        this.robotPoseStdDev = robotPoseStdDev;
    }

    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    public void setTimestampSeconds(double timestampSeconds) {
        this.timestampSeconds = timestampSeconds;
    }
}
