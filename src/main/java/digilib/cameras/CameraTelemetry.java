package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;

import static edu.wpi.first.units.Units.Seconds;
import static org.photonvision.PhotonPoseEstimator.*;

public class CameraTelemetry {
    private final DoublePublisher timestampPublisher;
    private final StructPublisher<Pose2d> robotPosePublisher;
    private final StructPublisher<Matrix<N3, N1>> robotPoseStdDevPublisher;
    private final StringPublisher cameraModePublisher;

    public CameraTelemetry(String name,
                           Transform3d robotToCamera,
                           PoseStrategy primaryStrategy,
                           PoseStrategy fallBackPoseStrategy) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getStructTopic("Robot to Camera", Transform3d.struct).publish().set(robotToCamera);
        table.getStringTopic("Primary Pose Strategy").publish().set(primaryStrategy.toString());
        table.getStringTopic("Fall Back Pose Strategy").publish().set(fallBackPoseStrategy.toString());
        timestampPublisher = table.getDoubleTopic("Timestamp").publish();
        robotPosePublisher = table.getStructTopic("Robot Pose", Pose2d.struct).publish();
        robotPoseStdDevPublisher = table.getStructTopic("Robot Pose Std Dev", Matrix.getStruct(Nat.N3(), Nat.N1())).publish();
        cameraModePublisher = table.getStringTopic("Camera mode").publish();
    }

    public void telemeterize(CameraState state) {
        timestampPublisher.set(state.getTimestamp().in(Seconds));
        robotPosePublisher.set(state.getRobotPose());
        robotPoseStdDevPublisher.set(state.getRobotPoseStdDev());
        cameraModePublisher.set(state.getCameraMode().toString());
    }
}
