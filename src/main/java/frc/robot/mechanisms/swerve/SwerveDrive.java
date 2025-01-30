package frc.robot.mechanisms.swerve;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
 */
public class SwerveDrive {

    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain;
    private final PhoenixPIDController pathXController;
    private final PhoenixPIDController pathYController;
    private final PhoenixPIDController pathThetaController;
    private final SwerveDriveTelemetry swerveDriveTelemetry;
    private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    public SwerveDrive(
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain,
            PhoenixPIDController pathXController,
            PhoenixPIDController pathYController,
            PhoenixPIDController pathThetaController,
            SwerveDriveTelemetry swerveDriveTelemetry) {
        this.swerveDriveTrain = swerveDriveTrain;
        this.pathXController = pathXController;
        this.pathYController = pathYController;
        this.pathThetaController = pathThetaController;
        this.swerveDriveTelemetry = swerveDriveTelemetry;
    }

    public void setControl(SwerveRequest request) {
        swerveDriveTrain.setControl(request);
    }

    public void updateSimState(double dtSeconds,
                               double supplyVoltage) {
        swerveDriveTrain.updateSimState(dtSeconds, supplyVoltage);
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = swerveDriveTrain.getState().Pose;
        var currentTimestamp = swerveDriveTrain.getState().Timestamp;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
                pose.getX(), sample.x, currentTimestamp
        );
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
                pose.getY(), sample.y, currentTimestamp
        );
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading, currentTimestamp
        );

        swerveDriveTrain.setControl(
                pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                        .withWheelForceFeedforwardsX(sample.moduleForcesX())
                        .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *                                 in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        swerveDriveTrain.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public void resetPose(Pose2d pose2d) {
        swerveDriveTrain.resetPose(pose2d);
    }

    public Pose2d getPose() {
        return swerveDriveTrain.getState().Pose;
    }

    public void setOperatorPerspectiveForward(Rotation2d rotation2d) {
        swerveDriveTrain.setOperatorPerspectiveForward(rotation2d);
    }

    public void seedFieldCentric() {
        swerveDriveTrain.seedFieldCentric();
    }

    public void updateTelemetry(){
        swerveDriveTelemetry.telemeterize(swerveDriveTrain.getState());
    }
}
