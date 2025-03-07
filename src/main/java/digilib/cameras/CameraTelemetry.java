package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CameraTelemetry {
    private final StringPublisher poseStrategyUsed;
    private final StringPublisher cameraModePublisher;
    private final IntegerPublisher bestFiducialIdPublisher;
    private final StructPublisher<Transform2d> bestTransformPublisher;
    private final double[] transformArray = new double[3];
    private final DoubleArrayPublisher bestTransformPublisherDashboard;
    private final StructPublisher<Pose2d> robotPosePublisher;
    private final double[] poseArray = new double[3];
    private final DoubleArrayPublisher robotPosePublisherDashboard;
    private final StructPublisher<Matrix<N3, N1>> robotPoseStdDevPublisher;
    private final double[] poseStdDevArray = new double[3];
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
                .getStructTopic("Best Transform", Transform2d.struct)
                .publish();
        bestTransformPublisherDashboard = table
                .getDoubleArrayTopic("Best Transform Array")
                .publish();
        robotPosePublisher = table
                .getStructTopic("Robot Pose", Pose2d.struct)
                .publish();
        robotPosePublisherDashboard = table
                .getDoubleArrayTopic("Robot Pose Array")
                .publish();
        robotPoseStdDevPublisher = table
                .getStructTopic("Robot Pose Std Dev", Matrix.getStruct(Nat.N3(), Nat.N1()))
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
        transformArray[2] = state.getBestTransformToFiducial().getRotation().getDegrees();
        bestTransformPublisherDashboard.set(transformArray);
        robotPosePublisher.set(state.getRobotPose());
        poseArray[0] = state.getRobotPose().getX();
        poseArray[1] = state.getRobotPose().getY();
        poseArray[2] = state.getRobotPose().getRotation().getDegrees();
        robotPosePublisherDashboard.set(poseArray);
        robotPoseStdDevPublisher.set(state.getRobotPoseStdDev());
        poseStdDevArray[0] = state.getRobotPoseStdDev().get(0, 0);
        poseStdDevArray[1] = state.getRobotPoseStdDev().get(1, 0);
        poseStdDevArray[2] = state.getRobotPoseStdDev().get(2, 0);
        robotPoseStdDevPublisherDashboard.set(poseStdDevArray);
        fieldPub.set(poseArray);
    }
}
