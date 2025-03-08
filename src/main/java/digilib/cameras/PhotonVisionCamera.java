package digilib.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static java.lang.Math.pow;

public class PhotonVisionCamera implements Camera {

    private static VisionSystemSim visionSim = null;
    private static boolean aprilTagsAdded = false;
    private static AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);


    private final CameraState state = new CameraState();
    private final Transform3d robotToCamera;
    private final CameraTelemetry telemetry;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevsTeleop;
    private final Matrix<N3, N1> multiTagStdDevsAuto;
    private PhotonCameraSim cameraSim = null;
    private Optional<Pose2d> estimatedPose = Optional.empty();
    private Optional<Matrix<N3, N1>> estimatedPoseStdDev = Optional.empty();


    public PhotonVisionCamera(CameraConstants constants, PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        this.robotToCamera = constants.robotToCamera();
        this.singleTagStdDevs = constants.singleTagStdDev();
        this.multiTagStdDevsTeleop = constants.multiTagStdDevTeleop();
        this.multiTagStdDevsAuto = constants.multiTagStdDevAuto();


        if (RobotBase.isSimulation()) {
            if (!aprilTagsAdded) {
                visionSim = new VisionSystemSim("main");
                visionSim.addAprilTags(constants.aprilTagFieldLayout());
                aprilTagsAdded = true;
            }
            SimCameraProperties simCameraProperties = new SimCameraProperties();
            simCameraProperties.setCalibration(
                    constants.xResolution(),
                    constants.yResolution(),
                    constants.fieldOfView());
            simCameraProperties.setCalibError(
                    constants.averageErrorPixels(),
                    constants.errorStdDevPixels());
            simCameraProperties.setFPS(constants.fps());
            simCameraProperties.setAvgLatencyMs(constants.averageLatencyMs());
            simCameraProperties.setLatencyStdDevMs(constants.latencyStdDevMs());
            cameraSim = new PhotonCameraSim(photonCamera, simCameraProperties);
            visionSim.addCamera(cameraSim, robotToCamera);
            cameraSim.enableDrawWireframe(true);
        }

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
    public void update() {
        List<PhotonPipelineResult> photonPipelineResults = photonCamera.getAllUnreadResults();
        Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        for (PhotonPipelineResult photonPipelineResult : photonPipelineResults) {
            estimatedRobotPose = photonPoseEstimator.update(photonPipelineResult);
        }
        List<PhotonTrackedTarget> photonTrackedTargets = photonPipelineResults.isEmpty()
                ? new ArrayList<>()
                : photonPipelineResults.get(photonPipelineResults.size() - 1).getTargets();
        Optional<Matrix<N3, N1>> estimatedRobotPoseStdDev = updateEstimatedGlobalPoseStdDev(estimatedRobotPose, photonTrackedTargets);
        updateState(estimatedRobotPose, estimatedRobotPoseStdDev);
        updateTelemetry();
    }


    private Optional<Matrix<N3, N1>> updateEstimatedGlobalPoseStdDev(Optional<EstimatedRobotPose> estimatedRobotPose, List<PhotonTrackedTarget> photonTrackedTargets) {
        if (estimatedRobotPose.isEmpty()) {
            return Optional.empty();
        }
        int numTags = 0;
        double avgDist = 0.0;
        for (PhotonTrackedTarget target : photonTrackedTargets) {
            Optional<Pose3d> tagPose = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedRobotPose.get().estimatedPose.toPose2d().getTranslation());
            }
        }

        avgDist /= numTags;

        if (avgDist > 6) {
            return Optional.empty();
        } else if (numTags > 1 && RobotModeTriggers.teleop().getAsBoolean()) {
            return Optional.of(multiTagStdDevsTeleop.times(1 + pow(avgDist, 2) / 5));
        } else {
            return Optional.of(multiTagStdDevsAuto.times(1 + pow(avgDist, 2) / 5));
        }
    }

    public void updateState(Optional<EstimatedRobotPose> estimatedRobotPose, Optional<Matrix<N3, N1>> estimatedRobotPoseStdDev) {
        state.setCameraMode(CameraState.CameraMode.ROBOT_POSE);
        state.setRobotPose(estimatedRobotPose);
        state.setRobotPoseStdDev(estimatedRobotPoseStdDev);
    }


    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /**
     * A Field2d for visualizing our robot and objects on the field.
     */
    private Field2d getSimDebugField() {
        if (!RobotBase.isSimulation()) {
            return null;
        }
        return visionSim.getDebugField();
    }
}
