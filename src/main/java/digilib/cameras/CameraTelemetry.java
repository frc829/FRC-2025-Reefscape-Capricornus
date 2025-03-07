package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CameraTelemetry {
    private final StringPublisher cameraModePublisher;
    private final StructPublisher<Pose3d> robotPosePublisher;
    private final double[] poseArray = new double[6];
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
        cameraModePublisher = table
                .getStringTopic("Camera mode")
                .publish();
        robotPosePublisher = table
                .getStructTopic("Robot Pose", Pose3d.struct)
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
        cameraModePublisher.set(state.getCameraMode());
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
        robotPoseStdDevPublisherDashboard.set(poseStdDevArray);
        fieldPub.set(poseArray);
    }
}
