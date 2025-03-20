package digilib.cameras;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static java.lang.Math.pow;

public class PhotonVisionCamera implements Camera {

    private final CameraState state = new CameraState();
    private final CameraTelemetry telemetry;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevs;

    public PhotonVisionCamera(CameraConstants constants, PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        photonPoseEstimator = new PhotonPoseEstimator(
                constants.aprilTagFieldLayout(),
                constants.primaryStrategy(),
                constants.robotToCamera());
        photonPoseEstimator.setMultiTagFallbackStrategy(constants.fallBackPoseStrategy());
        singleTagStdDevs = constants.singleTagStdDevs();
        multiTagStdDevs = constants.multiTagStdDevs();
        telemetry = new CameraTelemetry(
                constants.name(),
                constants.robotToCamera(),
                constants.primaryStrategy(),
                constants.fallBackPoseStrategy());
    }

    @Override
    public CameraState getState() {
        return state;
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    public void updateState() {
        // Get the unread camera results
        List<PhotonPipelineResult> photonPipelineResults = photonCamera.getAllUnreadResults();

        // update the photonPoseEstimator with the results
        Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        for (PhotonPipelineResult photonPipelineResult : photonPipelineResults) {
            estimatedRobotPose = photonPoseEstimator.update(photonPipelineResult);
        }

        // Get the photonTrackedTargets from the last result.
        List<PhotonTrackedTarget> photonTrackedTargets = photonPipelineResults.isEmpty()
                ? new ArrayList<>()
                : photonPipelineResults.get(photonPipelineResults.size() - 1).getTargets();

        // Set the estimatedRobotPose
        state.setEstimatedRobotPose(estimatedRobotPose.orElse(null));

        // Set estimatedRobotPoseStdDev
        if (estimatedRobotPose.isEmpty()) {
            state.setEstimatedRobotPoseStdDev(
                    MatBuilder.fill(
                            Nat.N3(),
                            Nat.N1(),
                            Double.MAX_VALUE,
                            Double.MAX_VALUE,
                            Double.MAX_VALUE));
            state.setNumberOfTagsUsedInPoseEstimate(0);
            state.setAverageTagDistanceMeters(Double.NaN);
        } else {
            int numTags = 0;
            double averageDistanceMeters = 0.0;
            for (PhotonTrackedTarget target : photonTrackedTargets) {
                Optional<Pose3d> tagPose = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    numTags++;
                    Translation2d tagTranslation2d = tagPose.get().toPose2d().getTranslation();
                    Translation2d estimatedRobotTranslation2d = estimatedRobotPose.get().estimatedPose.toPose2d().getTranslation();
                    double tagToRobotDistance = tagTranslation2d.getDistance(estimatedRobotTranslation2d);
                    averageDistanceMeters += tagToRobotDistance;
                }
            }
            state.setNumberOfTagsUsedInPoseEstimate(numTags);
            averageDistanceMeters /= numTags;
            state.setAverageTagDistanceMeters(averageDistanceMeters);

            if (averageDistanceMeters > 6.0) {
                state.setEstimatedRobotPoseStdDev(
                        MatBuilder.fill(
                                Nat.N3(),
                                Nat.N1(),
                                Double.MAX_VALUE,
                                Double.MAX_VALUE,
                                Double.MAX_VALUE));
                state.setSingleTagPoseAmbiguity(Double.MAX_VALUE);
            } else {
                if (numTags > 1) {
                    // multiplies the standard deviations by 1 + avgDist^2/5. this was found by
                    // others in the community to be an effective way to scale standard deviations
                    // by distance.
                    state.setEstimatedRobotPoseStdDev(multiTagStdDevs.times(1 + (pow(averageDistanceMeters, 2) / 5)));
                    state.setSingleTagPoseAmbiguity(0.0);
                } else {
                    state.setEstimatedRobotPoseStdDev(singleTagStdDevs.times(1 + (pow(averageDistanceMeters, 2) / 5)));
                    state.setSingleTagPoseAmbiguity(photonTrackedTargets.get(0).poseAmbiguity);
                }
            }
        }
    }


    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }
}
