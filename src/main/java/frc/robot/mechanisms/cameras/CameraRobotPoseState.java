package frc.robot.mechanisms.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

public class CameraRobotPoseState {

    private Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
    private Optional<Matrix<N3, N1>> estimatedRobotPoseStdDevs = Optional.empty();


    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public CameraRobotPoseState withEstimatedRobotPose(Optional<EstimatedRobotPose> estimatedRobotPose) {
        this.estimatedRobotPose = estimatedRobotPose;
        return this;
    }

    public CameraRobotPoseState withEstimatedRobotPoseStdDevs(Optional<Matrix<N3, N1>> estimatedRobotPoseStdDevs) {
        this.estimatedRobotPoseStdDevs = estimatedRobotPoseStdDevs;
        return this;
    }

    public CameraRobotPoseState withEstimatedRobotPoseState(CameraRobotPoseState cameraRobotPoseState) {
        this.estimatedRobotPose = cameraRobotPoseState.estimatedRobotPose;
        this.estimatedRobotPoseStdDevs = cameraRobotPoseState.estimatedRobotPoseStdDevs;
        return this;
    }




}
