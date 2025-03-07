package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.networktables.*;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CameraTelemetry {
    private final StringPublisher poseStrategyUsed;
    private final StringPublisher cameraModePublisher;
    private final IntegerPublisher bestFiducialIdPublisher;
    private final StructPublisher<Transform3d> bestTransformPublisher;
    private final double[] transformArray = new double[6];
    private final DoubleArrayPublisher bestTransformPublisherDashboard;
    private final StructPublisher<Pose3d> robotPosePublisher;
    private final double[] poseArray = new double[6];
    private final DoubleArrayPublisher robotPosePublisherDashboard;
    private final StructPublisher<Matrix<N6, N1>> robotPoseStdDevPublisher;
    private final double[] poseStdDevArray = new double[6];
    private final DoubleArrayPublisher robotPoseStdDevPublisherDashboard;
    private final DoubleArrayPublisher fieldPub;

    public CameraTelemetry(String name,
                           Transform3d robotToCamera,
                           PoseStrategy primaryStrategy,
                           PoseStrategy fallBackPoseStrategy) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        NetworkTable field = getDefault().getTable("Field");
        table.getStructTopic("Robot to Camera", Transform3d.struct)
                .publish()
                .set(robotToCamera);
        table.getStringTopic("Primary Pose Strategy")
                .publish()
                .set(primaryStrategy.toString());
        table.getStringTopic("Fall Back Pose Strategy")
                .publish()
                .set(fallBackPoseStrategy.toString());
        poseStrategyUsed = table
                .getStringTopic("Pose Strategy Used")
                .publish();
        cameraModePublisher = table
                .getStringTopic("Camera mode")
                .publish();
        bestFiducialIdPublisher = table
                .getIntegerTopic("Best Fiducial Id")
                .publish();
        bestTransformPublisher = table
                .getStructTopic("Best Transform", Transform3d.struct)
                .publish();
        bestTransformPublisherDashboard = table
                .getDoubleArrayTopic("Best Transform Array")
                .publish();
        robotPosePublisher = table
                .getStructTopic("Robot Pose", Pose3d.struct)
                .publish();
        robotPosePublisherDashboard = table
                .getDoubleArrayTopic("Robot Pose Array")
                .publish();
        robotPoseStdDevPublisher = table
                .getStructTopic("Robot Pose Std Dev", Matrix.getStruct(Nat.N6(), Nat.N1()))
                .publish();
        robotPoseStdDevPublisherDashboard = table
                .getDoubleArrayTopic("Robot Pose Std Dev Array")
                .publish();
        fieldPub = field
                .getDoubleArrayTopic(name + "-" + "Pose")
                .publish();
    }

    public void telemeterize(CameraState state) {
        poseStrategyUsed.set(state.getStrategyUsed());
        cameraModePublisher.set(state.getCameraMode());
        bestFiducialIdPublisher.set(state.getBestFiducialId());
        bestTransformPublisher.set(state.getBestTransformToFiducial());
        transformArray[0] = state.getBestTransformToFiducial().getX();
        transformArray[1] = state.getBestTransformToFiducial().getY();
        transformArray[2] = state.getBestTransformToFiducial().getZ();
        transformArray[3] = state.getBestTransformToFiducial().getRotation().getX();
        transformArray[4] = state.getBestTransformToFiducial().getRotation().getY();
        transformArray[5] = state.getBestTransformToFiducial().getRotation().getZ();
        bestTransformPublisherDashboard.set(transformArray);
        robotPosePublisher.set(state.getRobotPose());
        poseArray[0] = state.getRobotPose().getX();
        poseArray[1] = state.getRobotPose().getY();
        poseArray[2] = state.getRobotPose().getZ();
        poseArray[3] = state.getRobotPose().getRotation().getX();
        poseArray[4] = state.getRobotPose().getRotation().getY();
        poseArray[5] = state.getRobotPose().getRotation().getZ();
        robotPosePublisherDashboard.set(poseArray);
        robotPoseStdDevPublisher.set(state.getRobotPoseStdDev());
        poseStdDevArray[0] = state.getRobotPoseStdDev().get(0, 0);
        poseStdDevArray[1] = state.getRobotPoseStdDev().get(1, 0);
        poseStdDevArray[2] = state.getRobotPoseStdDev().get(2, 0);
        poseStdDevArray[3] = state.getRobotPoseStdDev().get(3, 0);
        poseStdDevArray[4] = state.getRobotPoseStdDev().get(4, 0);
        poseStdDevArray[5] = state.getRobotPoseStdDev().get(5, 0);
        robotPoseStdDevPublisherDashboard.set(poseStdDevArray);
        fieldPub.set(poseArray);
    }
}
