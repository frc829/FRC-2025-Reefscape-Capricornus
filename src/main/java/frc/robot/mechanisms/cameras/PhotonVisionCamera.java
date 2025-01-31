package frc.robot.mechanisms.cameras;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionCamera implements Camera{

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;  // TODO: this will be read until you do the item in the constructor
    private final CameraRobotPoseState cameraRobotPoseState = new CameraRobotPoseState();

    public PhotonVisionCamera(CameraConstants cameraConstants, PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        // TODO: assign a new PhotonPoseEstimator to the photonPoseEstimator field declared above.
        // TODO: you will need to pass in cameraConstants.getAprilTagFieldLayout(), cameraConstants.getPoseStrategy(), Constants.getCameraTransform()
        // TODO: call photonPoseEstimator's setMultiTagFallbackStrategy and pass in cameraConstants.getFallBackPoseStrategy()
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
        // TODO: create a List<PhotonPipelineResult> called results and assign from
        // photonCamera.getAllUnreadResults()
        // TODO: create an Optional<EstimatedRobotPose> named estimatedRobotPose and assign to Optional.empty()
        // TODO: create an Optional<Matrix<N3, N1>> named estimatedRobotPoseStdDev and assign to Optional.empty();
        // TODO: for each var result in results
        // TODO: for loop block start
        // TODO: set estimatedRobotPose to photonEstimator.update(result)
        // TODO: for loop block end.
        // TODO: call cameraRobotPoseState's withEstimatedRobotPose method passing in estimatedRobotPose
        // TODO: call cameraRobotPoseState's withEstimatedRobotPoseStdDev method passing in estimatedRobotPoseStdDev
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later.
    }


}
