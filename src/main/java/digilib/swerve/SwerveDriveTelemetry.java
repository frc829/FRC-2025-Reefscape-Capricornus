package digilib.swerve;

import choreo.trajectory.SwerveSample;
import digilib.DigiMath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

public class SwerveDriveTelemetry {

    private final DoubleArrayPublisher swerveSteerPositions;
    private final DoubleArrayPublisher swerveSteerVelocities;
    private final DoubleArrayPublisher swerveWheelPositions;
    private final DoubleArrayPublisher swerveWheelVelocities;
    private final StructPublisher<Pose2d> robotPose;
    private final DoubleArrayPublisher pose;
    private final StructPublisher<ChassisSpeeds> robotSpeeds;
    private final DoubleArrayPublisher speeds;
    private final StructArrayPublisher<SwerveModuleState> moduleStates;
    private final StructArrayPublisher<SwerveModuleState> moduleTargets;
    private final StructArrayPublisher<SwerveModulePosition> modulePositions;
    private final DoublePublisher rawHeading;
    private final double[] poseArray = new double[3];
    private final double[] speedsArray = new double[3];
    private final StructPublisher<SwerveSample> swerveSample;

    public SwerveDriveTelemetry(
            String name,
            double maxVelocityMPS,
            double maxAngularVelocityRPS) {
        NetworkTable table = getDefault().getTable(name);
        NetworkTable field = getDefault().getTable("Field");
        field.getStringTopic(".type").publish().set("Field2d");

        swerveSteerPositions = field
                .getDoubleArrayTopic("Swerve Steer Positions")
                .publish();
        swerveSteerVelocities = field
                .getDoubleArrayTopic("Swerve Steer Velocities")
                .publish();
        swerveWheelPositions = field
                .getDoubleArrayTopic("Swerve Wheel Positions")
                .publish();
        swerveWheelVelocities = field
                .getDoubleArrayTopic("Swerve Wheel Velocities")
                .publish();
        robotPose = field
                .getStructTopic("Robot", Pose2d.struct)
                .publish();
        robotSpeeds = field
                .getStructTopic("Robot Speeds", ChassisSpeeds.struct)
                .publish();
        moduleStates = field
                .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
                .publish();
        moduleTargets = field
                .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
                .publish();
        modulePositions = field
                .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
                .publish();

        table.getDoubleTopic("Max Velocity [mps]")
                .publish()
                .set(maxVelocityMPS);
        table.getDoubleTopic("Max Rotational Velocity [dps]")
                .publish()
                .set(maxAngularVelocityRPS * 360.0);
        rawHeading = table
                .getDoubleTopic("Raw Heading [deg]")
                .publish();
        pose = table
                .getDoubleArrayTopic("PoseArray")
                .publish();
        speeds = table
                .getDoubleArrayTopic("SpeedsArray")
                .publish();
        swerveSample = table
                .getStructTopic("Trajectory Samples", SwerveSample.struct)
                .publish();
    }

    public void telemeterize(SwerveDriveState state) {
        swerveSteerPositions.set(state.getSwerveSteerPositions());
        swerveSteerVelocities.set(state.getSwerveSteerVelocities());
        swerveWheelPositions.set(state.getSwerveWheelPositions());
        swerveWheelVelocities.set(state.getSwerveWheelVelocities());
        robotPose.set(state.getPose());
        robotSpeeds.set(state.getSpeeds());
        moduleStates.set(state.getModuleStates());
        moduleTargets.set(state.getModuleTargets());
        modulePositions.set(state.getModulePositions());

        poseArray[0] = DigiMath.roundToDecimal(state.getPose().getX(), 2);
        poseArray[1] = DigiMath.roundToDecimal(state.getPose().getY(), 2);
        poseArray[2] = DigiMath.roundToDecimal(state.getPose().getRotation().getDegrees(), 2);
        pose.set(poseArray);

        speedsArray[0] = DigiMath.roundToDecimal(state.getSpeeds().vxMetersPerSecond, 2);
        speedsArray[1] = DigiMath.roundToDecimal(state.getSpeeds().vyMetersPerSecond, 2);
        speedsArray[2] = DigiMath.roundToDecimal(state.getSpeeds().omegaRadiansPerSecond * 180 / Math.PI, 2);
        speeds.set(speedsArray);

        rawHeading.set(MathUtil.inputModulus(state.getRawHeading().getDegrees(), -180, 180));
        if(state.getSwerveSample() != null){
            swerveSample.set(state.getSwerveSample());
        }
    }
}
