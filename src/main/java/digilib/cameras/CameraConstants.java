package digilib.cameras;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;

import static org.photonvision.PhotonPoseEstimator.*;

public class CameraConstants {
    private final Transform3d cameraTransform;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PoseStrategy primaryStrategy;
    private final PoseStrategy fallBackPoseStrategy;

    // TODO: (Fake values. Experiment and determine estimation noise on an actual robot.)
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public CameraConstants(
            Distance cameraX,
            Distance cameraY,
            Distance cameraZ,
            Rotation3d rotation3d,
            AprilTagFieldLayout aprilTagFieldLayout,
            PoseStrategy primaryStrategy,
            PoseStrategy fallBackPoseStrategy) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.primaryStrategy = primaryStrategy;
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

    public PoseStrategy getPrimaryStrategy() {
        return primaryStrategy;
    }

    public PoseStrategy getFallBackPoseStrategy() {
        return fallBackPoseStrategy;
    }

    public Matrix<N3, N1> getSingleTagStdDevs() {
        return singleTagStdDevs;
    }

    public Matrix<N3, N1> getMultiTagStdDevs() {
        return multiTagStdDevs;
    }
}
