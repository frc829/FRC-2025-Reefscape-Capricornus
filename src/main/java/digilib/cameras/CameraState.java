package digilib.cameras;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

public class CameraState {

    private EstimatedRobotPose estimatedRobotPose = null;
    private final Matrix<N3, N1> estimatedRobotPoseStdDev = MatBuilder.fill(Nat.N3(),
            Nat.N1(),
            Double.MAX_VALUE,
            Double.MAX_VALUE,
            Double.MAX_VALUE);
    private double singleTagPoseAmbiguity = 0.0;
    private int numberOfTagsUsedInEstimate = 0;
    private double averageTagDistanceMeters = 0.0;

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return Optional.ofNullable(estimatedRobotPose);
    }

    public void setEstimatedRobotPose(EstimatedRobotPose estimatedRobotPose) {
        this.estimatedRobotPose = estimatedRobotPose;
    }

    public Matrix<N3, N1> getEstimatedRobotPoseStdDev() {
        return estimatedRobotPoseStdDev;
    }

    public void setEstimatedRobotPoseStdDev(Matrix<N3, N1> estimatedRobotPoseStdDev) {
        this.estimatedRobotPoseStdDev.assignBlock(0, 0, estimatedRobotPoseStdDev);
    }

    public double getSingleTagPoseAmbiguity() {
        return singleTagPoseAmbiguity;
    }

    public void setSingleTagPoseAmbiguity(double singleTagPoseAmbiguity) {
        this.singleTagPoseAmbiguity = singleTagPoseAmbiguity;
    }

    public int getNumberOfTagsUsedInEstimate() {
        return numberOfTagsUsedInEstimate;
    }

    public void setNumberOfTagsUsedInPoseEstimate(int numberOfTagsUsedInEstimate) {
        this.numberOfTagsUsedInEstimate = numberOfTagsUsedInEstimate;
    }

    public double getAverageTagDistanceMeters() {
        return averageTagDistanceMeters;
    }

    public void setAverageTagDistanceMeters(double averageTagDistanceMeters) {
        this.averageTagDistanceMeters = averageTagDistanceMeters;
    }
}