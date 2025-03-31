package digilib.swerve;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage;
import static com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue.OperatorPerspective;
import static java.lang.Math.*;

public class CTRESwerveDrive extends SwerveDrive {
    private final double maxVelocityMPS;
    private final double maxVelocityDeadband;
    private final double maxAngularVelocityRPS;
    private final double maxAngularVelocityDeadband;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain;

    private final FieldCentric fieldCentric = new FieldCentric()
            .withDriveRequestType(OpenLoopVoltage);
    private final RobotCentric robotCentric = new RobotCentric()
            .withDriveRequestType(OpenLoopVoltage);
    private final FieldCentricFacingAngle clockDrive = new FieldCentricFacingAngle()
            .withForwardPerspective(OperatorPerspective);
    private final Idle idle = new Idle();
    private final PointWheelsAt point = new PointWheelsAt();
    private final ApplyFieldSpeeds pathApplyFieldSpeeds = new ApplyFieldSpeeds();
    private final PhoenixPIDController clockThetaController;
    private final PhoenixPIDController pathXController;
    private final PhoenixPIDController pathYController;
    private final PhoenixPIDController pathThetaController;
    private SwerveSample swerveSample = null;
    private double pathXPositionError;
    private double pathYPositionError;
    private double pathThetaPositionError;
    private double pathXVelocityError;
    private double pathYVelocityError;
    private double pathThetaVelocityError;
    private double xVelocityCorrection;
    private double yVelocityCorrection;
    private double thetaCorrection;

    public CTRESwerveDrive(
            Config config,
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain) {
        super(config.name(), config.maxVelocityMPS(), config.maxAngularVelocityRPS());
        this.maxVelocityMPS = config.maxVelocityMPS();
        this.maxAngularVelocityRPS = config.maxAngularVelocityRPS();
        this.maxVelocityDeadband = config.maxVelocityDeadbandScalar();
        this.maxAngularVelocityDeadband = config.maxAngularVelocityDeadbandScalar();
        this.swerveDriveTrain = swerveDriveTrain;
        this.clockThetaController = config.clockThetaController();
        this.pathXController = config.pathXController();
        this.pathYController = config.pathYController();
        this.pathThetaController = config.pathThetaController();
        clockThetaController.enableContinuousInput(-Math.PI, Math.PI);
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        clockDrive.HeadingController = clockThetaController;
        pathXController.setTolerance(0.01, 0.01);
        pathYController.setTolerance(0.01, 0.01);
    }

    @Override
    public Pose2d getPose() {
        return swerveDriveTrain.getState().Pose;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return swerveDriveTrain.getState().Speeds;
    }

    @Override
    public ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getSpeeds(), swerveDriveTrain.getState().Pose.getRotation());
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return swerveDriveTrain.getState().ModuleStates;
    }

    @Override
    public SwerveModuleState[] getModuleTargets() {
        return swerveDriveTrain.getState().ModuleStates;
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return swerveDriveTrain.getState().ModulePositions;
    }

    @Override
    public Rotation2d getRawHeading() {
        return swerveDriveTrain.getState().RawHeading;
    }

    @Override
    public SwerveSample getSwerveSample() {
        if(swerveSample == null){
            return null;
        }
        return new SwerveSample(
                this.swerveSample.t,
                this.swerveSample.x,
                this.swerveSample.y,
                MathUtil.angleModulus(this.swerveSample.heading),
                this.swerveSample.vx,
                this.swerveSample.vy,
                this.swerveSample.omega,
                this.swerveSample.ax,
                this.swerveSample.ay,
                this.swerveSample.alpha,
                this.swerveSample.moduleForcesX(),
                this.swerveSample.moduleForcesY());
    }

    @Override
    public double getPathXPositionError() {
        return pathXPositionError;
    }

    public double getPathYPositionError() {
        return pathYPositionError;
    }

    public double getPathThetaPositionError() {
        return pathThetaPositionError;
    }

    public double getPathXVelocityError() {
        return pathXVelocityError;
    }

    public double getPathYVelocityError() {
        return pathYVelocityError;
    }

    public double getPathThetaVelocityError() {
        return pathThetaVelocityError;
    }

    @Override
    public double getXVelocityCorrection() {
        return xVelocityCorrection;
    }

    @Override
    public double getYVelocityCorrection() {
        return yVelocityCorrection;
    }

    @Override
    public double getThetaCorrection() {
        return thetaCorrection;
    }

    @Override
    public void setFieldCentric(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationalVelocitySetpointScalar) {
        fieldCentric
                .withVelocityX(cos(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS)
                .withVelocityY(sin(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS)
                .withRotationalRate(rotationalVelocitySetpointScalar * maxAngularVelocityRPS * 2 * Math.PI)
                .withDeadband(maxVelocityDeadband * maxVelocityMPS)
                .withRotationalDeadband(maxAngularVelocityDeadband * maxAngularVelocityRPS);
        swerveDriveTrain.setControl(fieldCentric);
    }

    @Override
    public void setRobotCentric(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationalVelocitySetpointScalar) {
        robotCentric
                .withVelocityX(cos(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS)
                .withVelocityY(sin(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS)
                .withRotationalRate(rotationalVelocitySetpointScalar * maxAngularVelocityRPS * 2 * Math.PI)
                .withDeadband(maxVelocityDeadband * maxVelocityMPS)
                .withRotationalDeadband(maxAngularVelocityDeadband * maxAngularVelocityRPS);
        swerveDriveTrain.setControl(robotCentric);
    }

    @Override
    public void setClockDrive(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationAngleDegrees) {
        clockDrive
                .withVelocityX(cos(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS)
                .withVelocityY(sin(toRadians(headingAngleDegrees)) * translationVelocitySetpointScalar * maxVelocityMPS)
                .withTargetDirection(Rotation2d.fromDegrees(rotationAngleDegrees))
                .withDeadband(maxVelocityDeadband * maxVelocityMPS)
                .withRotationalDeadband(maxAngularVelocityDeadband * maxAngularVelocityRPS);
        swerveDriveTrain.setControl(clockDrive);
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
        swerveSample = sample;
        var pose = swerveDriveTrain.getState().Pose;
        var currentTimestamp = swerveDriveTrain.getState().Timestamp;
        var targetSpeeds = sample.getChassisSpeeds();
        xVelocityCorrection = pathXController.calculate(
                pose.getX(), sample.x, currentTimestamp);
        yVelocityCorrection = pathYController.calculate(
                pose.getY(), sample.y, currentTimestamp);
        thetaCorrection = pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading, currentTimestamp);
        targetSpeeds.vxMetersPerSecond += xVelocityCorrection;
        targetSpeeds.vyMetersPerSecond += yVelocityCorrection;
        targetSpeeds.omegaRadiansPerSecond += thetaCorrection;

        pathXPositionError = pathXController.getPositionError();
        pathXVelocityError = pathXController.getVelocityError();
        pathYPositionError = pathYController.getPositionError();
        pathYVelocityError = pathYController.getVelocityError();
        pathThetaPositionError = pathThetaController.getPositionError();
        pathThetaVelocityError = pathThetaController.getVelocityError();

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
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        swerveDriveTrain.updateSimState(dtSeconds, supplyVoltage);
    }
}
