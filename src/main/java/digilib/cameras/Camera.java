package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

public interface Camera {

    CameraState getState();

    void update();

    void updateState(Optional<EstimatedRobotPose> estimatedRobotPose, Matrix<N3, N1> estimatedRobotPoseStdDev);

    void updateTelemetry();

    void updateSimState(Pose2d robotSimPose);
}
