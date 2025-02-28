package digilib.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static digilib.DigiMath.standardDeviation;
import static digilib.cameras.CameraState.*;
import static digilib.cameras.CameraState.CameraMode.*;

public class PhotonVisionCamera implements Camera {

    private final CameraState state = new CameraState();
    private final Transform3d robotToCamera;
    private final AprilTagFieldLayout fieldTags;
    private final Matrix<N3, N1> singleTagStdDev;
    private final String name;
    private CameraRequest cameraRequest;
    private final CameraTelemetry telemetry;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public PhotonVisionCamera(CameraConstants constants, PhotonCamera photonCamera) {
        this.name = constants.name();
        this.photonCamera = photonCamera;
        this.robotToCamera = constants.robotToCamera();
        this.fieldTags = constants.aprilTagFieldLayout();
        this.singleTagStdDev = constants.singleTagStdDev();
        photonPoseEstimator = new PhotonPoseEstimator(
                fieldTags,
                constants.primaryStrategy(),
                constants.robotToCamera());
        photonPoseEstimator.setMultiTagFallbackStrategy(constants.fallBackPoseStrategy());
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
            if (result.getBestTarget() != null) {
                if (result.multitagResult.isPresent()) {
                    Transform3d best = result.multitagResult.get().estimatedPose.best;
                    Transform3d fixedBest = new Transform3d(
                            best.getX(),
                            best.getY(),
                            best.getZ(),
                            new Rotation3d(
                                    best.getX(),
                                    best.getY(),
                                    best.getZ()
                            ));
                    state.setBestFiducialId(result.getBestTarget().fiducialId);
                    state.setBestTransformFiducialX(fixedBest.getX());
                    state.setBestTransformFiducialY(fixedBest.getY());
                    state.setBestTransformFiducialThetaRadians(fixedBest.getRotation().getZ() * 180 / Math.PI);
                    Pose3d pose3d = new Pose3d()
                            .plus(robotToCamera)
                            .plus(fixedBest)
                            .relativeTo(fieldTags.getOrigin());
                    updateEstimatedPoseFromMultiTag(pose3d, new ArrayList<PhotonTrackedTarget>(), result.getTimestampSeconds());
                } else if (result.getBestTarget().poseAmbiguity < 0.2) {
                    Transform3d best = result.getBestTarget().bestCameraToTarget;
                    Transform3d fixedBest = new Transform3d(
                            best.getX(),
                            best.getY(),
                            best.getZ(),
                            new Rotation3d(
                                    best.getX(),
                                    best.getY(),
                                    best.getZ()
                            ));
                    state.setBestFiducialId(result.getBestTarget().fiducialId);
                    state.setBestTransformFiducialX(fixedBest.getX());
                    state.setBestTransformFiducialY(fixedBest.getY());
                    state.setBestTransformFiducialThetaRadians(fixedBest.getRotation().getZ() * 180 / Math.PI);
                    Pose3d pose3d = new Pose3d()
                            .plus(robotToCamera)
                            .plus(fixedBest)
                            .relativeTo(fieldTags.getOrigin());
                    updateEstimatedPoseFromLowestAmbiguity(pose3d, new ArrayList<PhotonTrackedTarget>(), result.getTimestampSeconds());
                } else {
                    updateEstimatedPoseDefault();
                }
            }
        }
    }

    private void updateEstimatedPoseFromMultiTag(Pose3d pose3d, List<PhotonTrackedTarget> targets, double timestampSeconds) {
        state.setCameraMode(CameraMode.get(photonCamera.getPipelineIndex()));
        state.setRobotPose(pose3d.toPose2d());
        state.setTimestamp(timestampSeconds);
        updateRobotPoseStdDevFromMultiTag(pose3d.toPose2d(), targets);
    }

    private void updateRobotPoseStdDevFromMultiTag(Pose2d pose2d, List<PhotonTrackedTarget> targets) {
        List<Pose2d> targetPose2ds = targets.stream()
                .map(target -> new Pose3d()
                        .plus(robotToCamera)
                        .plus(target.bestCameraToTarget)
                        .relativeTo(fieldTags.getOrigin())
                        .toPose2d()).toList();

        double xStdDev = standardDeviation(
                pose2d.getX(),
                targetPose2ds
                        .stream()
                        .map(Pose2d::getX).toList());

        double yStdDev = standardDeviation(
                pose2d.getY(),
                targetPose2ds.stream()
                        .map(Pose2d::getY).toList());

        double thetaStdDevRadians = standardDeviation(
                pose2d.getRotation(),
                targetPose2ds.stream()
                        .map(Pose2d::getRotation).toList());

        state.setRobotPoseStdDev(xStdDev, yStdDev, thetaStdDevRadians);
    }

    private void updateEstimatedPoseFromLowestAmbiguity(Pose3d pose3d, List<PhotonTrackedTarget> targets, double timestampSeconds) {
        state.setCameraMode(CameraMode.get(photonCamera.getPipelineIndex()));
        state.setRobotPose(pose3d.toPose2d());
        state.setRobotPoseStdDev(singleTagStdDev.get(0, 0), singleTagStdDev.get(1, 0), singleTagStdDev.get(2, 0));
        state.setTimestamp(timestampSeconds);
    }

    private void updateEstimatedPoseDefault() {
        state.setCameraMode(CameraMode.get(photonCamera.getPipelineIndex()));
        state.setRobotPose(null);
        state.setRobotPoseStdDev(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        state.setTimestamp(Timer.getFPGATimestamp());
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }
}
