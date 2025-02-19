package digilib.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static org.photonvision.PhotonPoseEstimator.*;

public record CameraConstants(
        String name,
        Transform3d robotToCamera,
        AprilTagFieldLayout aprilTagFieldLayout,
        PoseStrategy primaryStrategy,
        PoseStrategy fallBackPoseStrategy,
        Matrix<N3, N1> singleTagStdDev) {
}
