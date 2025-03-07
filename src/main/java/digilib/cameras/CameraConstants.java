package digilib.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy;

public record CameraConstants(
        String name,
        Transform3d robotToCamera,
        AprilTagFieldLayout aprilTagFieldLayout,
        PoseStrategy primaryStrategy,
        PoseStrategy fallBackPoseStrategy,
        int xResolution,
        int yResolution,
        Rotation2d fieldOfView,
        double averageErrorPixels,
        double errorStdDevPixels,
        double fps,
        double averageLatencyMs,
        double latencyStdDevMs,
        Matrix<N3, N1> multiTagStdDevTeleop,
        Matrix<N3, N1> multiTagStdDevAuto,
        Matrix<N3, N1> singleTagStdDev) {
}
