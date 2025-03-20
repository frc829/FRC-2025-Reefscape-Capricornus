package digilib.cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.*;

import static digilib.DigiMath.roundToDecimal;
import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CameraTelemetry {
    private final StructPublisher<Pose2d> robotPosePublisher;
    private final DoubleArrayPublisher robotPose;
    private final DoubleArrayPublisher robotPoseStdDev;
    private final DoublePublisher singleTagPoseAmbiguity;
    private final IntegerPublisher numberOfTagsUsedInEstimate;
    private final DoublePublisher averageTagDistance;
    private final double[] poseArray = new double[3];
    private final double[] poseStdDevArray = new double[3];

    public CameraTelemetry(String name,
                           Transform3d robotToCamera,
                           PoseStrategy primaryStrategy,
                           PoseStrategy fallBackPoseStrategy) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        NetworkTable field = getDefault().getTable("Field");

        robotPosePublisher = field
                .getStructTopic(name + "-RobotPose", Pose2d.struct)
                .publish();
        table.getDoubleArrayTopic("Robot to Camera")
                .publish()
                .set(new double[]{
                        robotToCamera.getX(),
                        robotToCamera.getY(),
                        robotToCamera.getZ(),
                        roundToDecimal(robotToCamera.getRotation().getX(), 2),
                        roundToDecimal(robotToCamera.getRotation().getY(), 2),
                        roundToDecimal(robotToCamera.getRotation().getZ(), 2)});
        table.getStringTopic("Primary Pose Strategy")
                .publish()
                .set(primaryStrategy.toString());
        table.getStringTopic("Fall Back Pose Strategy")
                .publish()
                .set(fallBackPoseStrategy.toString());
        robotPose = table
                .getDoubleArrayTopic(name + "Robot Pose Array")
                .publish();
        robotPoseStdDev = table
                .getDoubleArrayTopic("Robot Pose Std Dev Array")
                .publish();
        singleTagPoseAmbiguity = table
                .getDoubleTopic("Single Tag Pose Ambiguity")
                .publish();
        numberOfTagsUsedInEstimate = table
                .getIntegerTopic("Number of Tags Used")
                .publish();
        averageTagDistance = table
                .getDoubleTopic("Average Tag Distance [m]")
                .publish();

    }

    public void telemeterize(CameraState state) {
        if(state.getEstimatedRobotPose().isPresent()) {
            robotPosePublisher.set(state.getEstimatedRobotPose().get().estimatedPose.toPose2d());

            poseArray[0] = roundToDecimal(state.getEstimatedRobotPose().get().estimatedPose.getX(), 2);
            poseArray[1] = roundToDecimal(state.getEstimatedRobotPose().get().estimatedPose.getY(), 2);
            poseArray[2] = roundToDecimal(state.getEstimatedRobotPose().get().estimatedPose.getRotation().getZ() * 180 / Math.PI, 2);
            robotPose.set(poseArray);
        }
        poseStdDevArray[0] = roundToDecimal(state.getEstimatedRobotPoseStdDev().get(0, 0), 2);
        poseStdDevArray[1] = roundToDecimal(state.getEstimatedRobotPoseStdDev().get(1, 0), 2);
        poseStdDevArray[2] = roundToDecimal(state.getEstimatedRobotPoseStdDev().get(2, 0), 2);
        robotPoseStdDev.set(poseStdDevArray);
        singleTagPoseAmbiguity.set(state.getSingleTagPoseAmbiguity());
        numberOfTagsUsedInEstimate.set(state.getNumberOfTagsUsedInEstimate());
        averageTagDistance.set(state.getAverageTagDistanceMeters());
    }
}