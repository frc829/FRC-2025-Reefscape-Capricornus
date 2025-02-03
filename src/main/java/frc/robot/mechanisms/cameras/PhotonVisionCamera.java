package frc.robot.mechanisms.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

public class PhotonVisionCamera implements Camera{

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final CameraRobotPoseState cameraRobotPoseState = new CameraRobotPoseState();

    public PhotonVisionCamera(CameraConstants cameraConstants, PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        photonPoseEstimator = new PhotonPoseEstimator(cameraConstants.getAprilTagFieldLayout(), cameraConstants.getPoseStrategy(), cameraConstants.getCameraTransform());
        photonPoseEstimator.setMultiTagFallbackStrategy(cameraConstants.getFallBackPoseStrategy());
    }

    @Override
    public CameraRobotPoseState getCameraRobotPoseState() {
        return cameraRobotPoseState;
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    public void updateState(){
        List <PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        Optional<Matrix<N3, N1>> estimatedRobotPoseStdDev = Optional.empty();
        for(var result : results){
            estimatedRobotPose = photonPoseEstimator.update(result);
        }
        cameraRobotPoseState.withEstimatedRobotPose(estimatedRobotPose);
        cameraRobotPoseState.withEstimatedRobotPoseStdDevs(estimatedRobotPoseStdDev);
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later.
    }


}
