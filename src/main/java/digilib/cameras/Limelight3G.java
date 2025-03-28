package digilib.cameras;

import digilib.cameras.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Limelight3G extends Camera {

    private final String limelightName;
    private final Matrix<N3, N1> robotPoseStdDev;
    private Pose2d pose2d = null;
    private double timestampSeconds = Double.NaN;
    private double poseAmbiguity = Double.MAX_VALUE;
    private Matrix<N3, N1> currentRobotPoseStdDev = null;
    private int tagCount = 0;

    public Limelight3G(Config config) {
        super(config);
        this.limelightName = config.name();
        this.robotPoseStdDev = config.robotPoseStdDev();
    }

    @Override
    public Pose2d getRobotPose() {
        return pose2d;
    }

    @Override
    public double getTimeStampSeconds() {
        return timestampSeconds;
    }

    @Override
    public double getRobotPoseAmbiguity() {
        return poseAmbiguity;
    }

    @Override
    public Matrix<N3, N1> getRobotPoseStdDev() {
        return currentRobotPoseStdDev;
    }

    @Override
    public int getTagCount() {
        return tagCount;
    }

    @Override
    public void setRobotOrientation(Pose2d robotOrientation) {
        LimelightHelpers.SetRobotOrientation(
                limelightName,
                robotOrientation.getRotation().getDegrees(),
                0,
                0,
                0,
                0,
                0);
    }

    @Override
    public void update() {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (poseEstimate != null) {
            pose2d = poseEstimate.pose;
            timestampSeconds = poseEstimate.timestampSeconds;
            poseAmbiguity = poseEstimate.rawFiducials.length == 1
                    ? poseEstimate.rawFiducials[0].ambiguity()
                    : Double.NaN;
            currentRobotPoseStdDev = robotPoseStdDev;
            tagCount = poseEstimate.tagCount;
        } else {
            pose2d = null;
            timestampSeconds = Double.NaN;
            poseAmbiguity = Double.MAX_VALUE;
            currentRobotPoseStdDev = null;
            tagCount = 0;
        }
        super.update();
    }
}
