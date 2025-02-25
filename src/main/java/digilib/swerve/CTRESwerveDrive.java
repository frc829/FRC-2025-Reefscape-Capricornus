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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import digilib.cameras.Camera;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

public class CTRESwerveDrive implements SwerveDrive {
    private final SwerveDriveState state = new SwerveDriveState();
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
    private final SwerveRequest.FieldCentricFacingAngle clockDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.SysIdSwerveTranslation driveCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.FieldCentric translationCharacterization = new SwerveRequest.FieldCentric();

    public CTRESwerveDrive(
            SwerveDriveConstants constants,
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain,
            Camera... cameras) {
        this.swerveDriveTrain = swerveDriveTrain;
        this.cameras = cameras;
        this.maxVelocity = constants.maxVelocity();
        this.maxAngularVelocity = constants.maxAngularVelocity();
        this.pathXController = constants.pathXController();
        this.pathYController = constants.pathYController();
        this.pathThetaController = constants.pathThetaController();
        this.swerveDriveTelemetry = new SwerveDriveTelemetry(constants.maxVelocity());
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
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

    public void setSteerCharacterization(Voltage voltage) {
        swerveDriveTrain.setControl(steerCharacterization.withVolts(voltage));
    }

    public void setDriveCharacterization(Voltage voltage) {
        swerveDriveTrain.setControl(driveCharacterization.withVolts(voltage));
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

    public void followPath(SwerveSample sample) {
        var pose = swerveDriveTrain.getState().Pose;
        var currentTimestamp = swerveDriveTrain.getState().Timestamp;
        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
                pose.getX(), sample.x, currentTimestamp);
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
                pose.getY(), sample.y, currentTimestamp);
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading, currentTimestamp);

        swerveDriveTrain.setControl(
                pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                        .withWheelForceFeedforwardsX(sample.moduleForcesX())
                        .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

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

    public void setOperatorPerspectiveForward(Rotation2d rotation2d) {
        swerveDriveTrain.setOperatorPerspectiveForward(rotation2d);
    }


    @Override
    public void update() {
        for (var camera : cameras) {
            CameraState state = camera.getState();
            if (camera.getState().getRobotPose() != null) {
                addVisionMeasurement(
                        state.getRobotPose(),
                        state.getTimestamp().in(Seconds),
                        state.getRobotPoseStdDev());
            }
        }
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.setPose(swerveDriveTrain.getState().Pose);
        state.setSpeeds(swerveDriveTrain.getState().Speeds);
        state.setModuleStates(swerveDriveTrain.getState().ModuleStates);
        state.setModuleTargets(swerveDriveTrain.getState().ModuleTargets);
        state.setModulePositions(swerveDriveTrain.getState().ModulePositions);
        state.setRawHeading(swerveDriveTrain.getState().RawHeading);
    }

    public void updateTelemetry() {
        swerveDriveTelemetry.telemeterize(state);
    }

    @Override
    public void updateSimState() {

    }
}
