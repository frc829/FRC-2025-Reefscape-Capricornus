package frc.robot.mechanisms.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class PhotonVisionCamera implements Camera {

    private final CameraState state = new CameraState();
    private final CameraState lastState = new CameraState();
    private CameraRequest cameraRequest;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevs;


    public PhotonVisionCamera(CameraConstants cameraConstants, PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        photonPoseEstimator = new PhotonPoseEstimator(
                cameraConstants.getAprilTagFieldLayout(),
                cameraConstants.getPrimaryStrategy(),
                cameraConstants.getCameraTransform());
        photonPoseEstimator.setMultiTagFallbackStrategy(cameraConstants.getFallBackPoseStrategy());
        singleTagStdDevs = cameraConstants.getSingleTagStdDevs();
        multiTagStdDevs = cameraConstants.getMultiTagStdDevs();
    }

    @Override
    public void setControl(CameraRequest request) {
        if (cameraRequest != request) {
            cameraRequest = request;
        }
        request.apply(this);

    }

    @Override
    public CameraState getState() {
        return state;
    }

    @Override
    public CameraState getStateCopy() {
        return state.clone();
    }

    @Override
    public CameraState getLastState() {
        return lastState;
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later.
    }

    @Override
    public void setMode(CameraState.CameraMode mode) {
        // TODO: will do later.
    }


    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    public void updateState() {
        lastState.withCameraState(state);
        for (var change : photonCamera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(change);
            if (estimatedRobotPose.isPresent()) {
                Matrix<N3, N1> stdDevs = updateEstimationStdDevs(estimatedRobotPose.get(), change.getTargets());
            }
        }
    }

    private Matrix<N3, N1> updateEstimationStdDevs(
            EstimatedRobotPose estimatedRobotPose,
            List<PhotonTrackedTarget> targets) {
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose
                            .get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(estimatedRobotPose.estimatedPose.toPose2d().getTranslation());
        }
    }


}
