package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;

import static digilib.DigiMath.roundToDecimal;
import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CameraTelemetry {
    private final StringPublisher cameraModePublisher;
    private final StructPublisher<Pose3d> robotPosePublisher;
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
        if (state.getRobotPose().isPresent()) {
            robotPosePublisher.set(state.getRobotPose().get().estimatedPose);
            poseArray[0] = roundToDecimal(state.getRobotPose().get().estimatedPose.getX(), 2);
            poseArray[1] = roundToDecimal(state.getRobotPose().get().estimatedPose.getY(), 2);
            poseArray[2] = roundToDecimal(state.getRobotPose().get().estimatedPose.getRotation().getZ() * 180 / Math.PI, 2);
            robotPosePublisherDashboard.set(poseArray);
            fieldPub.set(poseArray);
        }else{
            robotPosePublisherDashboard.set(new double[0]);
        }


        if (state.getRobotPoseStdDev().isPresent()) {
            robotPoseStdDevPublisher.set(state.getRobotPoseStdDev().get());
            poseStdDevArray[0] = roundToDecimal(state.getRobotPoseStdDev().get().get(0, 0), 2);
            poseStdDevArray[1] = roundToDecimal(state.getRobotPoseStdDev().get().get(1, 0), 2);
            poseStdDevArray[2] = roundToDecimal(state.getRobotPoseStdDev().get().get(2, 0), 2);
            robotPoseStdDevPublisherDashboard.set(poseStdDevArray);
        }else{
            robotPoseStdDevPublisherDashboard.set(new double[0]);
        }

    }
}