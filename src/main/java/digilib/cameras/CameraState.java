package digilib.cameras;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

import java.util.Map;
import java.util.Optional;

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
    private Optional<EstimatedRobotPose> robotPose = Optional.empty();
    private Optional<Matrix<N3, N1>> robotPoseStdDev = Optional.of(MatBuilder
            .fill(Nat.N3(),
                    Nat.N1(),
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE));

    public String getCameraMode() {
        return cameraMode;
    }

    public void setCameraMode(CameraMode cameraMode) {
        this.cameraMode = cameraMode.name();
    }

    public Optional<EstimatedRobotPose> getRobotPose() {
        return robotPose;
    }

    public void setRobotPose(Optional<EstimatedRobotPose> robotPose) {
        this.robotPose = robotPose;
    }

    public Optional<Matrix<N3, N1>> getRobotPoseStdDev() {
        return robotPoseStdDev;
    }

    public void setRobotPoseStdDev(Optional<Matrix<N3, N1>> robotPoseStdDev) {
        this.robotPoseStdDev = robotPoseStdDev;
    }
}