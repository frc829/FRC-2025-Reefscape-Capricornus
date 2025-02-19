package digilib.cameras;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;

import java.util.Map;
import java.util.Optional;

import static edu.wpi.first.units.Units.Seconds;

public class CameraState {

    public enum CameraMode {
        ROBOT_POSE(0);

        private final int pipelineIndex;

        CameraMode(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        private static Map<Integer, CameraMode> map = Map.of(
                0, ROBOT_POSE);


        public int getPipelineIndex() {
            return pipelineIndex;
        }

        public static CameraMode get(int pipelineIndex) {
            return map.get(pipelineIndex);
        }
    }

    private CameraMode cameraMode = null;
    private Pose3d robotPose = new Pose3d(
            Double.NaN,
            Double.NaN,
            Double.NaN,
            new Rotation3d(Double.NaN, Double.NaN, Double.NaN));
    private Matrix<N3, N1> robotPoseStdDev = MatBuilder.fill(
            Nat.N3(),
            Nat.N1(),
            Double.NaN,
            Double.NaN,
            Double.NaN);
    private final MutTime timestamp = Seconds.mutable(0.0);

    public CameraMode getCameraMode() {
        return cameraMode;
    }

    public Pose3d getRobotPose() {
        return robotPose;
    }

    public Matrix<N3, N1> getRobotPoseStdDev() {
        return robotPoseStdDev;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public CameraState withCameraMode(CameraMode cameraMode) {
        this.cameraMode = cameraMode;
        return this;
    }

    public CameraState withRobotPose(Pose3d robotPose) {
        this.robotPose = robotPose;
        return this;
    }

    public CameraState withTimestamp(double seconds) {
        timestamp.mut_setBaseUnitMagnitude(seconds);
        return this;
    }
}
