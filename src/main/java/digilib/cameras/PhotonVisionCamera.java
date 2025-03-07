package digilib.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

public class PhotonVisionCamera implements Camera {

    private static VisionSystemSim visionSim = null;
    private static boolean aprilTagsAdded = false;
    private static AprilTagFieldLayout fieldTags = null;


    private final CameraState state = new CameraState();
    private final Transform3d robotToCamera;
    private final CameraTelemetry telemetry;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevsTeleop;
    private final Matrix<N3, N1> multiTagStdDevsAuto;
    private PhotonCameraSim cameraSim = null;


    public PhotonVisionCamera(CameraConstants constants, PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        this.robotToCamera = constants.robotToCamera();
        this.singleTagStdDevs = constants.singleTagStdDev();
        this.multiTagStdDevsTeleop = constants.multiTagStdDevTeleop();
        this.multiTagStdDevsAuto = constants.multiTagStdDevAuto();

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

        if (RobotBase.isSimulation()) {
            if (!aprilTagsAdded) {
                visionSim = new VisionSystemSim("main");
                visionSim.addAprilTags(constants.aprilTagFieldLayout());
                PhotonVisionCamera.fieldTags = constants.aprilTagFieldLayout();
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
            if (estimatedRobotPose.isPresent()) {
                if (estimatedRobotPose.get().estimatedPose.getZ() <= -1 || estimatedRobotPose.get().estimatedPose.getZ() >= 1) {
                    estimatedRobotPose = Optional.empty();
                }
            }

            if (RobotBase.isSimulation()) {
                estimatedRobotPose.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject(photonCamera.getName() + "-" + "VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject(photonCamera.getName() + "-" + "VisionEstimation").setPoses();
                        });
            }
        }
        List<PhotonTrackedTarget> photonTrackedTargets = photonPipelineResults.isEmpty()
                ? new ArrayList<>()
                : photonPipelineResults.get(photonPipelineResults.size() - 1).getTargets();
        Matrix<N3, N1> estimatedRobotPoseStdDev = updateEstimatedGlobalPoseStdDev(estimatedRobotPose, photonTrackedTargets);
        updateState(estimatedRobotPose, estimatedRobotPoseStdDev);
        updateTelemetry();
    }


    private Matrix<N3, N1> updateEstimatedGlobalPoseStdDev(Optional<EstimatedRobotPose> estimatedRobotPose, List<PhotonTrackedTarget> photonTrackedTargets) {
        if (estimatedRobotPose.isEmpty()) {
            return MatBuilder.fill(Nat.N3(), Nat.N1(), Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
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


        return null;
    }

    public void updateState(Optional<EstimatedRobotPose> estimatedRobotPose, Matrix<N3, N1> estimatedRobotPoseStdDev) {

    }


    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState() {

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
