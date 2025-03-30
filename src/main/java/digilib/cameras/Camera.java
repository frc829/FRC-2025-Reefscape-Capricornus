package digilib.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;

import java.util.Arrays;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

public abstract class Camera {

    /**
     * Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output.
     */
    public record RawFiducial(
            int id,
            double txnc,
            double tync,
            double ta,
            double distToCamera,
            double distToRobot,
            double ambiguity) {

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            RawFiducial other = (RawFiducial) obj;
            return id == other.id &&
                    Double.compare(txnc, other.txnc) == 0 &&
                    Double.compare(tync, other.tync) == 0 &&
                    Double.compare(ta, other.ta) == 0 &&
                    Double.compare(distToCamera, other.distToCamera) == 0 &&
                    Double.compare(distToRobot, other.distToRobot) == 0 &&
                    Double.compare(ambiguity, other.ambiguity) == 0;
        }
    }

    /**
     * Represents a 3D Pose Estimate.
     */
    public record PoseEstimate(
            Pose2d pose,
            double timestampSeconds,
            double latency,
            int tagCount,
            double tagSpan,
            double avgTagDist,
            double avgTagArea,
            RawFiducial[] rawFiducials) {

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            PoseEstimate that = (PoseEstimate) obj;
            // We don't compare the timestampSeconds as it isn't relevant for equality and makes
            // unit testing harder
            return Double.compare(that.latency, latency) == 0
                    && tagCount == that.tagCount
                    && Double.compare(that.tagSpan, tagSpan) == 0
                    && Double.compare(that.avgTagDist, avgTagDist) == 0
                    && Double.compare(that.avgTagArea, avgTagArea) == 0
                    && pose.equals(that.pose)
                    && Arrays.equals(rawFiducials, that.rawFiducials);
        }
    }

    public record Config(
            String name,
            Transform3d robotToCamera,
            Matrix<N3, N1> robotPoseStdDev) {
    }

    private final StructPublisher<Pose2d> robotPose;
    private final StructPublisher<Matrix<N3, N1>> robotPoseStdDev;
    private final DoublePublisher robotPoseAmbiguity;
    private final DoublePublisher timeStampSeconds;
    private final IntegerPublisher tagCount;

    public Camera(Config config) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(config.name());
        NetworkTable field = getDefault().getTable("Field");

        table.getDoubleArrayTopic("Robot to Camera")
                .publish()
                .set(new double[]{
                        config.robotToCamera().getX(),
                        config.robotToCamera().getY(),
                        config.robotToCamera().getZ(),
                        config.robotToCamera().getRotation().getX(),
                        config.robotToCamera().getRotation().getY(),
                        config.robotToCamera().getRotation().getZ()});
        robotPose = field
                .getStructTopic(config.name() + "-RobotPose", Pose2d.struct)
                .publish();
        robotPoseStdDev = table
                .getStructTopic("Robot Pose Std Dev", Matrix.getStruct(Nat.N3(), Nat.N1()))
                .publish();
        robotPoseAmbiguity = table
                .getDoubleTopic("Single Tag Pose Ambiguity")
                .publish();
        timeStampSeconds = table
                .getDoubleTopic("TimeStamp [seconds]")
                .publish();
        tagCount = table
                .getIntegerTopic("Tag Count")
                .publish();
    }

    public abstract Pose2d getRobotPose();

    public abstract Pose2d getRobotPoseMT1();

    public abstract double getTimeStampSeconds();

    public abstract double getRobotPoseAmbiguity();

    public abstract Matrix<N3, N1> getRobotPoseStdDev();

    public abstract int getTagCount();

    public abstract void setRobotOrientation(Pose2d robotOrientation);

    public void update() {
        robotPose.set(getRobotPose());
        robotPoseStdDev.set(getRobotPoseStdDev());
        timeStampSeconds.set(getTimeStampSeconds());
        robotPoseAmbiguity.set(getRobotPoseAmbiguity());
        tagCount.set(getTagCount());
    }
}