package digilib.cameras;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Map;

import java.util.concurrent.ConcurrentHashMap;

import static digilib.cameras.Camera.*;

/**
 * LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
 * This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
 */
public class LimelightHelpers {

    private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

    static PoseEstimate getBotPoseEstimateMT2(String limelightName) {
        DoubleArrayEntry poseEntry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "botpose_orb_wpiblue");
        return getBotPose(poseEntry);
    }

    static PoseEstimate getBotPoseEstimateMT1(String limelightName) {
        DoubleArrayEntry poseEntry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "botpose_wpiblue");
        return getBotPose(poseEntry);
    }

    public static PoseEstimate getBotPose(DoubleArrayEntry poseEntry){
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;

        if (poseArray.length == 0) {
            // Handle the case where no data is available
            return null; // or some default PoseEstimate
        }

        Translation2d translation2d = new Translation2d(poseArray[0], poseArray[1]);
        Rotation2d rotation2d = new Rotation2d(Units.degreesToRadians(poseArray[5]));
        Pose2d pose = new Pose2d(translation2d, rotation2d);
        double latency = poseArray[6];
        int tagCount = (int) poseArray[7];
        double tagSpan = poseArray[8];
        double tagDist = poseArray[9];
        double tagArea = poseArray[10];

        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length == expectedTotalVals) {
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials);
    }

    public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
        String key = tableName + "/" + entryName;
        return doubleArrayEntries.computeIfAbsent(key, k -> NetworkTableInstance
                .getDefault()
                .getTable("".equals(tableName) || tableName == null ? "limelight" : tableName)
                .getDoubleArrayTopic(entryName).getEntry(new double[0]));
    }

    /**
     * Sets robot orientation values used by MegaTag2 localization algorithm.
     *
     * @param limelightName Name/identifier of the Limelight
     * @param yaw           Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
     */
    public static void SetRobotOrientation(String limelightName, double yaw) {
        double[] entries = new double[6];
        entries[0] = yaw;
        entries[1] = 0;
        entries[2] = 0;
        entries[3] = 0;
        entries[4] = 0;
        entries[5] = 0;
        NetworkTableInstance.getDefault()
                .getTable("".equals(limelightName) || limelightName == null
                        ? "limelight"
                        : limelightName)
                .getEntry("robot_orientation_set")
                .setValue(entries);
        NetworkTableInstance.getDefault().flush();
    }
}