package frc.robot.mechanisms.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import org.photonvision.PhotonPoseEstimator;

import static org.photonvision.PhotonPoseEstimator.*;

public class CameraConstants {
    private final Transform3d cameraTransform;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PoseStrategy poseStrategy;
    private final PoseStrategy fallBackPoseStrategy;

    public CameraConstants(
            Distance cameraX,
            Distance cameraY,
            Distance cameraZ,
            Rotation3d rotation3d,
            AprilTagFieldLayout aprilTagFieldLayout,
            PoseStrategy poseStrategy,
            PoseStrategy fallBackPoseStrategy) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.poseStrategy = poseStrategy;
        this.fallBackPoseStrategy = fallBackPoseStrategy;
        cameraTransform = new Transform3d(
                cameraX.baseUnitMagnitude(),
                cameraY.baseUnitMagnitude(),
                cameraZ.baseUnitMagnitude(),
                rotation3d);
    }

    public Transform3d getCameraTransform() {
        return cameraTransform;
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagFieldLayout;
    }

    public PoseStrategy getPoseStrategy() {
        return poseStrategy;
    }

    public PoseStrategy getFallBackPoseStrategy() {
        return fallBackPoseStrategy;
    }
}
