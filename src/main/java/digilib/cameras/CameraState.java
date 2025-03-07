package digilib.cameras;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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

    private String strategyUsed = "N/A";
    private String cameraMode = "N/A";
    private int bestFiducialId = 0;
    private Transform3d bestTransformToFiducial = new Transform3d(Double.NaN, Double.NaN, Double.NaN, new Rotation3d(Double.NaN, Double.NaN, Double.NaN));
    private Pose3d robotPose = new Pose3d(Double.NaN, Double.NaN, Double.NaN, new Rotation3d(Double.NaN, Double.NaN, Double.NaN));
    private final Matrix<N6, N1> robotPoseStdDev = MatBuilder
            .fill(Nat.N6(),
                    Nat.N1(),
                    Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY);
    private double timestampSeconds = 0.0;

    public String getStrategyUsed() {
        return strategyUsed;
    }

    public void setStrategyUsed(PoseStrategy strategyUsed) {
        this.strategyUsed = strategyUsed.name();
    }

    public String getCameraMode() {
        return cameraMode;
    }

    public void setCameraMode(CameraMode cameraMode) {
        this.cameraMode = cameraMode.name();
    }


    public int getBestFiducialId() {
        return bestFiducialId;
    }

    public void setBestFiducialId(int bestFiducialId) {
        this.bestFiducialId = bestFiducialId;
    }

    public Transform3d getBestTransformToFiducial() {
        return bestTransformToFiducial;
    }

    public void setBestTransformToFiducial(Transform3d bestTransformToFiducial) {
        this.bestTransformToFiducial = bestTransformToFiducial;
    }

    public Pose3d getRobotPose() {
        return robotPose;
    }

    public void setRobotPose(Pose3d robotPose) {
        this.robotPose = robotPose;
    }

    public Matrix<N6, N1> getRobotPoseStdDev() {
        return robotPoseStdDev;
    }

    public void setRobotPoseStdDev(
            double xMetersStdDev,
            double yMetersStdDev,
            double zMetersStdDev,
            double xAngleRadiansStdDev,
            double yAngleRadiansStdDev,
            double zAngleRadiansStdDev) {
        this.robotPoseStdDev.set(0, 0, xMetersStdDev);
        this.robotPoseStdDev.set(1, 0, yMetersStdDev);
        this.robotPoseStdDev.set(2, 0, zMetersStdDev);
        this.robotPoseStdDev.set(3, 0, xAngleRadiansStdDev);
        this.robotPoseStdDev.set(4, 0, yAngleRadiansStdDev);
        this.robotPoseStdDev.set(5, 0, zAngleRadiansStdDev);
    }

    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    public void setTimestampSeconds(double timestampSeconds) {
        this.timestampSeconds = timestampSeconds;
    }
}
