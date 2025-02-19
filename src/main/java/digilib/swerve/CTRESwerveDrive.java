package digilib.swerve;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import digilib.cameras.CameraState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import digilib.cameras.Camera;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

public class CTRESwerveDrive implements SwerveDrive {

    private final PhoenixPIDController pathXController;
    private final PhoenixPIDController pathYController;
    private final PhoenixPIDController pathThetaController;
    private final SwerveDriveTelemetry swerveDriveTelemetry;
    private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final LinearVelocity maxVelocity;
    private final AngularVelocity maxAngularVelocity;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain;
    private final Camera[] cameras;
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);     // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);     // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();   // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.FieldCentricFacingAngle clockDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.SysIdSwerveTranslation driveCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.FieldCentric translationCharacterization = new SwerveRequest.FieldCentric();

    public CTRESwerveDrive(
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain,
            PhoenixPIDController pathXController,
            PhoenixPIDController pathYController,
            PhoenixPIDController pathThetaController,
            SwerveDriveTelemetry swerveDriveTelemetry,
            LinearVelocity maxVelocity,
            AngularVelocity maxAngularVelocity,
            Camera... cameras) {
        this.swerveDriveTrain = swerveDriveTrain;
        this.pathXController = pathXController;
        this.pathYController = pathYController;
        this.pathThetaController = pathThetaController;
        this.swerveDriveTelemetry = swerveDriveTelemetry;
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.cameras = cameras;
        clockDrive.HeadingController = pathThetaController;
    }

    public LinearVelocity getMaxVelocity() {
        return maxVelocity;
    }

    public AngularVelocity getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public void setControl(SwerveDriveRequest request) {
        request.apply(this);
    }

    public void setFieldCentric(double vx, double vy, double omega) {
        swerveDriveTrain.setControl(fieldCentric.withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega));
    }

    public void setRobotCentric(double vx, double vy, double omega) {
        swerveDriveTrain.setControl(robotCentric.withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega));
    }

    public void setBrake() {
        swerveDriveTrain.setControl(brake);
    }

    public void setWheels(Angle direction) {
        swerveDriveTrain.setControl(point.withModuleDirection(
                Rotation2d.fromDegrees(direction.in(Degrees))));
    }

    public void setFieldCentricSeed() {
        swerveDriveTrain.seedFieldCentric();
    }

    public void setClockDrive(double vx, double vy, double thetaRadians) {
        swerveDriveTrain.setControl(
                clockDrive
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(Rotation2d.fromRadians(thetaRadians))
                        .withDeadband(0.1 * maxVelocity.baseUnitMagnitude()));
    }

    public void setIdle() {
        swerveDriveTrain.setControl(idle);
    }

    public void setDriveCharacterization(Voltage voltage) {
        swerveDriveTrain.setControl(driveCharacterization.withVolts(voltage));
    }

    public void setSteerCharacterization(Voltage voltage) {
        swerveDriveTrain.setControl(steerCharacterization.withVolts(voltage));
    }

    public void setRotationCharacterization(AngularVelocity newRotationalRate) {
        swerveDriveTrain.setControl(rotationCharacterization.withRotationalRate(newRotationalRate));
    }

    public void setTranslationCharacterization(LinearVelocity velocity) {
        swerveDriveTrain.setControl(translationCharacterization
                .withVelocityX(velocity.baseUnitMagnitude())
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
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
            Matrix<N3, N1> visionMeasurementStdDevs) {
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

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDriveTrain.getState().Speeds;
    }

    public void setOperatorPerspectiveForward(Rotation2d rotation2d) {
        swerveDriveTrain.setOperatorPerspectiveForward(rotation2d);
    }


    @Override
    public void update() {
        for (var camera : cameras) {
            CameraState state = camera.getState();
            if (Double.isFinite(camera.getState().getRobotPose().getX())) {
                addVisionMeasurement(
                        state.getRobotPose(),
                        state.getTimestamp().in(Seconds),
                        state.getRobotPoseStdDev());
            }
        }
    }

    @Override
    public void updateState() {

    }

    public void updateTelemetry() {
        swerveDriveTelemetry.telemeterize(swerveDriveTrain.getState());
    }

    @Override
    public void updateSimState() {

    }
}
