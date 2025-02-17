package digilib.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import static digilib.cameras.CameraState.*;
import static digilib.cameras.CameraState.CameraMode.*;

public class PhotonVisionCamera implements Camera {

    private final CameraState state = new CameraState();
    private final Transform3d cameraToRobot;
    private final AprilTagFieldLayout fieldTags;
    private final Matrix<N3, N1> singleTagStdDev;
    private CameraRequest cameraRequest;
    // private final CameraTelemetry telemetry;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public PhotonVisionCamera(CameraConstants constants, PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        this.cameraToRobot = constants.robotToCamera().inverse();
        this.fieldTags = constants.aprilTagFieldLayout();
        this.singleTagStdDev = constants.singleTagStdDev();
        photonPoseEstimator = new PhotonPoseEstimator(
                fieldTags,
                constants.primaryStrategy(),
                constants.robotToCamera());
        photonPoseEstimator.setMultiTagFallbackStrategy(constants.fallBackPoseStrategy());
        // telemetry = new CameraTelemetry(
        //         constants.name(),
        //         constants.robotToCamera(),
        //         constants.primaryStrategy(),
        //         constants.fallBackPoseStrategy());
    }

    @Override
    public CameraState getState() {
        return state;
    }

    @Override
    public void setControl(CameraRequest request) {
        if (cameraRequest != request) {
            cameraRequest = request;
        }
        request.apply(this);
    }

    @Override
    public void setRobotPoseMode() {
        photonCamera.setPipelineIndex(ROBOT_POSE.getPipelineIndex());
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    public void updateState() {
        List<PhotonPipelineResult> photonPipelineResults = photonCamera.getAllUnreadResults();
        for (PhotonPipelineResult result : photonPipelineResults) {
            Optional<EstimatedRobotPose> optionalEstimatedRobotPose = photonPoseEstimator.update(result);
            optionalEstimatedRobotPose.ifPresentOrElse(
                    estimatedRobotPose -> {
                        switch (estimatedRobotPose.strategy) {
                            case MULTI_TAG_PNP_ON_COPROCESSOR -> updateEstimatedPoseFromMultiTag(estimatedRobotPose);
                            case LOWEST_AMBIGUITY -> updateEstimatedPoseFromLowestAmbiguity(estimatedRobotPose);
                            default -> updateEstimatedPoseDefault();
                        }
                    }, this::updateEstimatedPoseDefault);
        }
    }

    private void updateEstimatedPoseFromMultiTag(EstimatedRobotPose estimatedRobotPose) {
        state.withCameraMode(CameraMode.get(photonCamera.getPipelineIndex()))
                .withRobotPose(estimatedRobotPose.estimatedPose.toPose2d())
                .withTimestamp(estimatedRobotPose.timestampSeconds);
        updateRobotPoseStdDevFromMultiTag(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.targetsUsed);
    }

    private void updateRobotPoseStdDevFromMultiTag(Pose2d pose2d, List<PhotonTrackedTarget> targets) {
        List<Pose2d> targetPose2ds = targets.stream()
                .map(target -> new Pose3d()
                        .plus(target.bestCameraToTarget)
                        .relativeTo(fieldTags.getOrigin())
                        .plus(cameraToRobot)
                        .toPose2d()).toList();

        double xStdDev = targetPose2ds
                .stream()
                .map(targetPose -> Math.pow(targetPose.getX() - pose2d.getX(), 2))
                .reduce(0.0, Double::sum)
                / targetPose2ds.size();
        xStdDev = Math.sqrt(xStdDev);

        double yStdDev = targetPose2ds
                .stream()
                .map(targetPose -> Math.pow(targetPose.getY() - pose2d.getY(), 2))
                .reduce(0.0, Double::sum)
                / targetPose2ds.size();
        yStdDev = Math.sqrt(yStdDev);

        double thetaStdDevRadians = targetPose2ds
                .stream()
                .map(targetPose -> targetPose.getRotation().minus(pose2d.getRotation()))
                .map(rotation2d -> Math.pow(rotation2d.getRadians(), 2))
                .reduce(0.0, Double::sum)
                / targetPose2ds.size();
        state.withRobotPoseStdDev(xStdDev, yStdDev, thetaStdDevRadians);
    }

    private void updateEstimatedPoseFromLowestAmbiguity(EstimatedRobotPose estimatedRobotPose) {
        if (estimatedRobotPose.targetsUsed.get(0).poseAmbiguity <= 0.2) {
            state.withCameraMode(CameraMode.get(photonCamera.getPipelineIndex()))
                    .withRobotPose(estimatedRobotPose.estimatedPose.toPose2d())
                    .withRobotPoseStdDev(singleTagStdDev.get(0, 0), singleTagStdDev.get(1, 0), singleTagStdDev.get(2, 0))
                    .withTimestamp(estimatedRobotPose.timestampSeconds);
        } else {
            updateEstimatedPoseDefault();
        }
    }

    private void updateEstimatedPoseDefault() {
        state.withCameraMode(CameraMode.get(photonCamera.getPipelineIndex()))
                .withRobotPose(null)
                .withRobotPoseStdDev(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)
                .withTimestamp(Timer.getFPGATimestamp());
    }

    @Override
    public void updateTelemetry() {
        // telemetry.telemeterize(state);
    }
}
