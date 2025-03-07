package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    private Transform2d bestTransformToFiducial = new Transform2d(Double.NaN, Double.NaN, Rotation2d.fromRadians(Double.NaN));
    private Pose2d robotPose = new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromDegrees(Double.NaN));
    private final Matrix<N3, N1> robotPoseStdDev = new Matrix<>(Nat.N3(), Nat.N1());
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

    public Transform2d getBestTransformToFiducial() {
        return bestTransformToFiducial;
    }

    public void setBestTransformToFiducial(Transform2d bestTransformToFiducial) {
        this.bestTransformToFiducial = bestTransformToFiducial;
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public void setRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }

    public Matrix<N3, N1> getRobotPoseStdDev() {
        return robotPoseStdDev;
    }

    public void setRobotPoseStdDev(double xMetersStdDev, double yMetersStdDev, double thetaRadiansStdDev) {
        this.robotPoseStdDev.set(0, 0, xMetersStdDev);
        this.robotPoseStdDev.set(1, 0, yMetersStdDev);
        this.robotPoseStdDev.set(2, 0, thetaRadiansStdDev);
    }

    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    public void setTimestampSeconds(double timestampSeconds) {
        this.timestampSeconds = timestampSeconds;
    }
}
