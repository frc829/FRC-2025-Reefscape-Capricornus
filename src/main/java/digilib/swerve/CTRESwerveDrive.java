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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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
            double translationalVelocityMPS,
            double headingAngleDegrees,
            double rotationalVelocityDPS) {
        double vx = cos(toRadians(headingAngleDegrees)) * translationalVelocityMPS * maxVelocityMPS;
        double vy = sin(toRadians(headingAngleDegrees)) * translationalVelocityMPS * maxVelocityMPS;
        double omega = toRadians(rotationalVelocityDPS) * maxAngularVelocityRPS;
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
            double translationalVelocityMPS,
            double headingAngleDegrees,
            double rotationalVelocityDPS) {
        double vx = cos(toRadians(headingAngleDegrees)) * translationalVelocityMPS * maxVelocityMPS;
        double vy = sin(toRadians(headingAngleDegrees)) * translationalVelocityMPS * maxVelocityMPS;
        double omega = toRadians(rotationalVelocityDPS) * maxAngularVelocityRPS;
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
            double translationalVelocityMPS,
            double headingAngleDegrees,
            double rotationAngleDegrees) {
        double vx = cos(toRadians(headingAngleDegrees)) * translationalVelocityMPS * maxVelocityMPS;
        double vy = sin(toRadians(headingAngleDegrees)) * translationalVelocityMPS * maxVelocityMPS;
        clockDrive
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withTargetDirection(Rotation2d.fromDegrees(rotationAngleDegrees))
                .withDeadband(maxVelocityDeadband * maxVelocityMPS)
                .withRotationalDeadband(maxAngularVelocityDeadband * maxAngularVelocityRPS);
        swerveDriveTrain.setControl(clockDrive);
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
        swerveDriveTrain.resetPose(pose2d);
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
