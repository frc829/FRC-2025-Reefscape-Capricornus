package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;

import static digilib.DigiMath.roundToDecimal;
import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CameraTelemetry {
    private final StructPublisher<Pose2d> robotPosePublisher;
    private final StructPublisher<Matrix<N3, N1>> robotPoseStdDevPublisher;

    private final StringPublisher cameraModePublisher;
    private final DoubleArrayPublisher robotPose;
    private final DoubleArrayPublisher robotPoseStdDev;
    private final double[] poseArray = new double[3];
    private final double[] poseStdDevArray = new double[3];

    public CameraTelemetry(String name,
                           Transform3d robotToCamera,
                           PoseStrategy primaryStrategy,
                           PoseStrategy fallBackPoseStrategy) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        NetworkTable field = getDefault().getTable("Field");

        robotPosePublisher = field
                .getStructTopic(name + "-RobotPose", Pose2d.struct)
                .publish();
        robotPoseStdDevPublisher = table
                .getStructTopic(name + "-Robot Pose Std Dev", Matrix.getStruct(Nat.N3(), Nat.N1()))
                .publish();

        table.getDoubleArrayTopic("Robot to Camera")
                .publish()
                .set(new double[]{
                        robotToCamera.getX(),
                        robotToCamera.getY(),
                        robotToCamera.getZ(),
                        roundToDecimal(robotToCamera.getRotation().getX(), 2),
                        roundToDecimal(robotToCamera.getRotation().getY(), 2),
                        roundToDecimal(robotToCamera.getRotation().getZ(), 2)});
        table.getStringTopic("Primary Pose Strategy")
                .publish()
                .set(primaryStrategy.toString());
        table.getStringTopic("Fall Back Pose Strategy")
                .publish()
                .set(fallBackPoseStrategy.toString());
        cameraModePublisher = table
                .getStringTopic("Camera mode")
                .publish();
        robotPose = table
                .getDoubleArrayTopic(name + "Robot Pose Array")
                .publish();
        robotPoseStdDev = table
                .getDoubleArrayTopic("Robot Pose Std Dev Array")
                .publish();
    }

    public void telemeterize(CameraState state) {
        if(state.getRobotPose().isPresent() && state.getRobotPoseStdDev().isPresent()) {
            robotPosePublisher.set(state.getRobotPose().get().estimatedPose.toPose2d());
            robotPoseStdDevPublisher.set(state.getRobotPoseStdDev().get());

            poseArray[0] = roundToDecimal(state.getRobotPose().get().estimatedPose.getX(), 2);
            poseArray[1] = roundToDecimal(state.getRobotPose().get().estimatedPose.getY(), 2);
            poseArray[2] = roundToDecimal(state.getRobotPose().get().estimatedPose.getRotation().getZ() * 180 / Math.PI, 2);
            robotPose.set(poseArray);

            poseStdDevArray[0] = roundToDecimal(state.getRobotPoseStdDev().get().get(0, 0), 2);
            poseStdDevArray[1] = roundToDecimal(state.getRobotPoseStdDev().get().get(1, 0), 2);
            poseStdDevArray[2] = roundToDecimal(state.getRobotPoseStdDev().get().get(2, 0), 2);
            robotPoseStdDev.set(poseStdDevArray);
        }
        cameraModePublisher.set(state.getCameraMode());
    }
}