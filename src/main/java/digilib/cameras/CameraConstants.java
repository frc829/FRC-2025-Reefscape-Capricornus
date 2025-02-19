package digilib.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

import static org.photonvision.PhotonPoseEstimator.*;

public record CameraConstants(
        String name,
        Transform3d robotToCamera,
        AprilTagFieldLayout aprilTagFieldLayout,
        PoseStrategy primaryStrategy,
        PoseStrategy fallBackPoseStrategy) {
}
