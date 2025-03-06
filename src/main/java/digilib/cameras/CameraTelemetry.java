package digilib.cameras;

import digilib.DigiMath;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.*;

import static edu.wpi.first.units.Units.Seconds;
import static org.photonvision.PhotonPoseEstimator.*;

public class CameraTelemetry {
    private final IntegerPublisher bestFiducialIdPublisher;
    private final DoublePublisher bestTransformFiducialXPublisher;
    private final DoublePublisher bestTransformFiducialYPublisher;
    private final DoublePublisher bestTransformFiducialThetaRadiansPublisher;
    private final DoublePublisher timestampPublisher;
    private final DoubleArrayPublisher robotPosePublisher;
    private final DoubleArrayPublisher robotPoseStdDevPublisher;
    private final StringPublisher cameraModePublisher;

    private final DoubleArrayPublisher fieldPub;
    private final StringPublisher fieldTypePub;
    private final double[] poseArray = new double[3];
    private final double[] poseStdDevArray = new double[3];


    public CameraTelemetry(String name,
                           Transform3d robotToCamera,
                           PoseStrategy primaryStrategy,
                           PoseStrategy fallBackPoseStrategy) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getStructTopic("Robot to Camera", Transform3d.struct).publish().set(robotToCamera);
        table.getStringTopic("Primary Pose Strategy").publish().set(primaryStrategy.toString());
        table.getStringTopic("Fall Back Pose Strategy").publish().set(fallBackPoseStrategy.toString());

        bestFiducialIdPublisher = table.getIntegerTopic("Best Fiducial Id").publish();
        bestTransformFiducialXPublisher = table.getDoubleTopic("Best Transform Fiducial X").publish();
        bestTransformFiducialYPublisher = table.getDoubleTopic("Best Transform Fiducial Y").publish();
        bestTransformFiducialThetaRadiansPublisher = table.getDoubleTopic("Best Transform Fiducial Theta").publish();
        timestampPublisher = table.getDoubleTopic("Timestamp").publish();
        robotPosePublisher = table.getDoubleArrayTopic("Robot Pose").publish();
        robotPoseStdDevPublisher = table.getDoubleArrayTopic("Robot Pose Std Dev").publish();
        cameraModePublisher = table.getStringTopic("Camera mode").publish();


        NetworkTable poseTable = NetworkTableInstance.getDefault().getTable("Pose");
        fieldPub = poseTable.getDoubleArrayTopic(name + "robotPose").publish();
        fieldTypePub = poseTable.getStringTopic(".type").publish();
    }

    public void telemeterize(CameraState state) {
        bestFiducialIdPublisher.set(state.getBestFiducialId());
        bestTransformFiducialXPublisher.set(state.getBestFiducialTransformX());
        bestTransformFiducialYPublisher.set(state.getBestFiducialTransformY());
        bestTransformFiducialThetaRadiansPublisher.set(state.getBestFiducialTransformThetaRadians());
        timestampPublisher.set(state.getTimestamp().in(Seconds));
        cameraModePublisher.set(state.getCameraMode() != null ? state.getCameraMode().toString() : "null");

        if (state.getRobotPose() != null) {
            poseArray[0] = DigiMath.roundToDecimal(state.getRobotPose().getX(), 2);
            poseArray[1] = DigiMath.roundToDecimal(state.getRobotPose().getY(), 2);
            poseArray[2] = DigiMath.roundToDecimal(state.getRobotPose().getRotation().getDegrees(), 2);

            robotPosePublisher.set(new double[]{poseArray[0], poseArray[1], poseArray[2]});
        }

        if(state.getRobotPoseStdDev() != null){
            poseStdDevArray[0] = DigiMath.roundToDecimal(state.getRobotPoseStdDev().get(0, 0), 2);
            poseStdDevArray[1] = DigiMath.roundToDecimal(state.getRobotPoseStdDev().get(1, 0), 2);
            poseStdDevArray[2] = DigiMath.roundToDecimal(state.getRobotPoseStdDev().get(2, 0) * 180 / Math.PI, 2);
            robotPoseStdDevPublisher.set(new double[]{poseStdDevArray[0], poseStdDevArray[1], poseStdDevArray[2]});

        }


        fieldTypePub.set("Field2d");
        fieldPub.set(poseArray);
    }
}
