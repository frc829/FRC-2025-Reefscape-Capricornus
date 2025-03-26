package digilib.swerve;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Arrays;
import java.util.stream.Collectors;

import static com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage;
import static com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue.OperatorPerspective;
import static java.lang.Math.*;

public class CTRESwerveDrive implements SwerveDrive {
    private final SwerveDriveState state = new SwerveDriveState();
    private final double maxVelocityMPS;
    private final double maxVelocityDeadband;
    private final double maxAngularVelocityRPS;
    private final double maxAngularVelocityDeadband;
    private final SwerveDriveTelemetry swerveDriveTelemetry;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain;

    private final FieldCentric fieldCentric = new FieldCentric()
            .withDriveRequestType(OpenLoopVoltage);
    private final RobotCentric robotCentric = new RobotCentric()
            .withDriveRequestType(OpenLoopVoltage);
    private final FieldCentricFacingAngle clockDrive = new FieldCentricFacingAngle()
            .withForwardPerspective(OperatorPerspective);
    private final SwerveDriveBrake brake = new SwerveDriveBrake();
    private final Idle idle = new Idle();
    private final PointWheelsAt point = new PointWheelsAt();
    private final ApplyFieldSpeeds pathApplyFieldSpeeds = new ApplyFieldSpeeds();
    private final PhoenixPIDController pathXController;
    private final PhoenixPIDController pathYController;
    private final PhoenixPIDController pathThetaController;

    public CTRESwerveDrive(
            SwerveDriveConstants constants,
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain) {
        this.maxVelocityMPS = constants.maxVelocityMPS();
        this.maxAngularVelocityRPS = constants.maxAngularVelocityRPS();
        this.maxVelocityDeadband = constants.maxVelocityDeadbandScalar();
        this.maxAngularVelocityDeadband = constants.maxAngularVelocityDeadbandScalar();
        this.swerveDriveTrain = swerveDriveTrain;
        this.swerveDriveTelemetry = new SwerveDriveTelemetry(
                constants.name(),
                constants.maxVelocityMPS(),
                constants.maxAngularVelocityRPS());
        this.pathXController = constants.pathXController();
        this.pathYController = constants.pathYController();
        this.pathThetaController = constants.pathThetaController();
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        clockDrive.HeadingController = pathThetaController;
    }

    @Override
    public double getMaxVelocityMPS() {
        return maxVelocityMPS;
    }

    @Override
    public double getMaxAngularVelocityRPS() {
        return maxAngularVelocityRPS;
    }

    @Override
    public SwerveDriveState getState() {
        return state;
    }

    @Override
    public void setFieldCentric(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationalVelocitySetpointScalar) {
        double vx = cos(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS;
        double vy = sin(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS;
        double omega = rotationalVelocitySetpointScalar * maxAngularVelocityRPS * 2 * Math.PI;
        fieldCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega)
                .withDeadband(maxVelocityDeadband * maxVelocityMPS)
                .withRotationalDeadband(maxAngularVelocityDeadband * maxAngularVelocityRPS);
        swerveDriveTrain.setControl(fieldCentric);
    }

    @Override
    public void setRobotCentric(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationalVelocitySetpointScalar) {
        double vx = cos(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS;
        double vy = sin(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS;
        double omega = rotationalVelocitySetpointScalar * maxAngularVelocityRPS * 2 * Math.PI;
        robotCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega)
                .withDeadband(maxVelocityDeadband * maxVelocityMPS)
                .withRotationalDeadband(maxAngularVelocityDeadband * maxAngularVelocityRPS);
        swerveDriveTrain.setControl(robotCentric);
    }

    @Override
    public void setClockDrive(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationAngleDegrees) {
        double vx = cos(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS;
        double vy = sin(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS;
        clockDrive
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withTargetDirection(Rotation2d.fromDegrees(rotationAngleDegrees))
                .withDeadband(maxVelocityDeadband * maxVelocityMPS)
                .withRotationalDeadband(maxAngularVelocityDeadband * maxAngularVelocityRPS);
        swerveDriveTrain.setControl(clockDrive);
    }

    @Override
    public void setSteerAngle(double angleDegrees) {
        point.withModuleDirection(Rotation2d.fromDegrees(angleDegrees));
        swerveDriveTrain.setControl(point);
    }

    @Override
    public void setBrake() {
        swerveDriveTrain.setControl(brake);
    }

    @Override
    public void setIdle() {
        swerveDriveTrain.setControl(idle);
    }

    @Override
    public void setWheelAngle(double wheelAngleDegrees) {
        point.withModuleDirection(Rotation2d.fromDegrees(wheelAngleDegrees));
        swerveDriveTrain.setControl(point);
    }

    @Override
    public void followPath(SwerveSample sample) {
        state.setSwerveSample(sample);
        var pose = swerveDriveTrain.getState().Pose;
        var currentTimestamp = swerveDriveTrain.getState().Timestamp;
        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
                pose.getX(), sample.x, currentTimestamp);
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
                pose.getY(), sample.y, currentTimestamp);
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading, currentTimestamp);
        pathApplyFieldSpeeds
                .withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY());
        swerveDriveTrain.setControl(pathApplyFieldSpeeds);
    }

    public void rotateInPlace(Rotation2d angle) {
        var pose = swerveDriveTrain.getState().Pose;
        var currentTimeStamp = swerveDriveTrain.getState().Timestamp;
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
        speeds.omegaRadiansPerSecond += pathThetaController.calculate(
                pose.getRotation().getRadians(), angle.getRadians(), currentTimeStamp);
        pathApplyFieldSpeeds
                .withSpeeds(speeds);
        swerveDriveTrain.setControl(pathApplyFieldSpeeds);
    }

    public void goToPose(Pose2d pose2d) {
        var pose = swerveDriveTrain.getState().Pose;
        var currentTimestamp = swerveDriveTrain.getState().Timestamp;
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
        speeds.vxMetersPerSecond += pathXController.calculate(
                pose.getX(), pose2d.getX(), currentTimestamp);
        speeds.vyMetersPerSecond += pathYController.calculate(
                pose.getY(), pose2d.getY(), currentTimestamp);
        speeds.omegaRadiansPerSecond += pathThetaController.calculate(
                pose.getRotation().getRadians(), pose2d.getRotation().getRadians(), currentTimestamp);
        pathApplyFieldSpeeds
                .withSpeeds(speeds);
        swerveDriveTrain.setControl(pathApplyFieldSpeeds);
    }

    @Override
    public void seedFieldCentric() {
        swerveDriveTrain.seedFieldCentric();
    }

    @Override
    public void setOperatorPerspectiveForward(Rotation2d rotation2d) {
        swerveDriveTrain.setOperatorPerspectiveForward(rotation2d);
    }

    @Override
    public void resetPose(Pose2d pose2d) {
        if (pose2d != null) {
            swerveDriveTrain.resetPose(pose2d);
        }
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        swerveDriveTrain.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    @Override
    public void update() {
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
        state.setSwerveSteerPositions(
                Arrays.stream(swerveDriveTrain.getModules())
                        .map(module -> module.getSteerMotor().getPosition().getValueAsDouble())
                        .toList());
        state.setSwerveSteerVelocities(
                Arrays.stream(swerveDriveTrain.getModules())
                        .map(module -> module.getSteerMotor().getVelocity().getValueAsDouble())
                        .toList());
        state.setSwerveWheelPositions(
                Arrays.stream(swerveDriveTrain.getModules())
                        .map(module -> module.getDriveMotor().getPosition().getValueAsDouble())
                        .toList());
        state.setSwerveWheelVelocities(
                Arrays.stream(swerveDriveTrain.getModules())
                        .map(module -> module.getDriveMotor().getVelocity().getValueAsDouble())
                        .toList());
    }

    @Override
    public void updateTelemetry() {
        swerveDriveTelemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        swerveDriveTrain.updateSimState(dtSeconds, supplyVoltage);
    }
}
