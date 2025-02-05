package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
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
        Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        Matrix<N3, N1> estimatedRobotPoseStdDev;
        for (var change : photonCamera.getAllUnreadResults()) {
            estimatedRobotPose = photonPoseEstimator.update(change);
            if (estimatedRobotPose.isPresent()) {
                var pose = estimatedRobotPose.get();
                var targets = change.getTargets();
                int numTags = 0;
                double avgDist = 0;

                // Precalculation - see how many tags we found, and calculate an average-distance metric
                for (var tgt : targets) {
                    Optional<Pose3d> tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if (tagPose.isPresent()) {
                        numTags++;
                        avgDist +=
                                tagPose
                                        .get()
                                        .toPose2d()
                                        .getTranslation()
                                        .getDistance(pose.estimatedPose.toPose2d().getTranslation());
                    }
                }

                if (numTags == 0) {
                    // No tags visible. Default to single-tag std devs
                    estimatedRobotPoseStdDev = singleTagStdDevs;
                } else {
                    // One or more tags visible, run the full heuristic.
                    avgDist /= numTags;
                    // Decrease std devs if multiple targets are visible
                    if (numTags > 1) estimatedRobotPoseStdDev = multiTagStdDevs;
                    // Increase std devs based on (average) distance
                    if (numTags == 1 && avgDist > 4)
                        estimatedRobotPoseStdDev = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    else estimatedRobotPoseStdDev = singleTagStdDevs.times(1 + (avgDist * avgDist / 30));
                }
            }
        }
    }

    private Matrix<N3, N1> updateEstimationStdDevs(
            EstimatedRobotPose estimatedRobotPose,
            List<PhotonTrackedTarget> targets) {
        return null;
    }


}
