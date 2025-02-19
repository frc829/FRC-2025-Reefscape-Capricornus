package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;

import java.util.Map;

import static edu.wpi.first.units.Units.Seconds;

public class CameraState {

    public enum CameraMode {
        ROBOT_POSE(0);

        private final int pipelineIndex;

        CameraMode(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        private static final Map<Integer, CameraMode> map = Map.of(
                0, ROBOT_POSE);


        public int getPipelineIndex() {
            return pipelineIndex;
        }

        public static CameraMode get(int pipelineIndex) {
            return map.get(pipelineIndex);
        }
    }

    private CameraMode cameraMode = null;
    private Pose2d robotPose = null;
    private final Matrix<N3, N1> robotPoseStdDev = new Matrix<>(Nat.N3(), Nat.N1());
    private final MutTime timestamp = Seconds.mutable(0.0);

    public CameraMode getCameraMode() {
        return cameraMode;
    }

    public Pose2d getRobotPose() {
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

    public CameraState withRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
        return this;
    }

    public CameraState withRobotPoseStdDev(double xStdDev, double yStdDev, double thetaStdDev) {
        this.robotPoseStdDev.set(0, 0, xStdDev);
        this.robotPoseStdDev.set(1, 0, yStdDev);
        this.robotPoseStdDev.set(2, 0, thetaStdDev);
        return this;
    }

    public CameraState withTimestamp(double seconds) {
        timestamp.mut_setBaseUnitMagnitude(seconds);
        return this;
    }
}
