package digilib.swerve;

import choreo.trajectory.SwerveSample;
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
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

public abstract class SwerveDrive {

    public record Config(
            String name,
            double maxVelocityMPS,
            double maxAngularVelocityRPS,
            double maxVelocityDeadbandScalar,
            double maxAngularVelocityDeadbandScalar,
            PhoenixPIDController clockThetaController,
            PhoenixPIDController pathXController,
            PhoenixPIDController pathYController,
            PhoenixPIDController pathThetaController) {
    }

    private final StructPublisher<Pose2d> robotPose;
    private final StructPublisher<ChassisSpeeds> robotSpeeds;
    private final StructPublisher<ChassisSpeeds> fieldSpeeds;
    private final StructArrayPublisher<SwerveModuleState> moduleStates;
    private final StructArrayPublisher<SwerveModuleState> moduleTargets;
    private final StructArrayPublisher<SwerveModulePosition> modulePositions;
    private final DoublePublisher rawHeading;
    private final StructPublisher<SwerveSample> swerveSample;
    private final DoublePublisher pathXPositionError;
    private final DoublePublisher pathYPositionError;
    private final DoublePublisher pathThetaPositionError;
    private final DoublePublisher pathXVelocityError;
    private final DoublePublisher pathYVelocityError;
    private final DoublePublisher pathThetaVelocityError;
    private final DoublePublisher xVelocityCorrection;
    private final DoublePublisher yVelocityCorrection;
    private final DoublePublisher thetaCorrection;

    public SwerveDrive(
            String name,
            double maxVelocityMPS,
            double maxAngularVelocityRPS){
        NetworkTable table = getDefault().getTable(name);
        NetworkTable field = getDefault().getTable("Field");
        field.getStringTopic(".type").publish().set("Field2d");

        table.getDoubleTopic("Max Velocity [mps]")
                .publish()
                .set(maxVelocityMPS);
        table.getDoubleTopic("Max Rotational Velocity [dps]")
                .publish()
                .set(maxAngularVelocityRPS * 360.0);
        robotPose = field
                .getStructTopic("Robot", Pose2d.struct)
                .publish();
        fieldSpeeds = table
                .getStructTopic("Field Speeds", ChassisSpeeds.struct)
                .publish();
        robotSpeeds = table
                .getStructTopic("Robot Speeds", ChassisSpeeds.struct)
                .publish();
        moduleStates = table
                .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
                .publish();
        moduleTargets = table
                .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
                .publish();
        modulePositions = table
                .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
                .publish();
        rawHeading = table
                .getDoubleTopic("Raw Heading [deg]")
                .publish();
        swerveSample = table
                .getStructTopic("Trajectory Samples", SwerveSample.struct)
                .publish();
        pathXPositionError = table
                .getDoubleTopic("Path X Position Error [m]")
                .publish();
        pathYPositionError = table
                .getDoubleTopic("Path Y Position Error [m]")
                .publish();
        pathThetaPositionError = table
                .getDoubleTopic("Path Theta Position Error [deg]")
                .publish();
        pathXVelocityError = table
                .getDoubleTopic("Path X Velocity Error [mps]")
                .publish();
        pathYVelocityError = table
                .getDoubleTopic("Path Y Velocity Error [mps]")
                .publish();
        pathThetaVelocityError = table
                .getDoubleTopic("Path Theta Velocity Error [dps]")
                .publish();
        xVelocityCorrection = table
                .getDoubleTopic("X Velocity Correction [mps")
                .publish();
        yVelocityCorrection = table
                .getDoubleTopic("Y Velocity Correction [mps")
                .publish();
        thetaCorrection = table
                .getDoubleTopic("Theta Velocity Correction [dps]")
                .publish();
    }

    public abstract Pose2d getPose();

    public abstract ChassisSpeeds getFieldSpeeds();

    public abstract ChassisSpeeds getSpeeds();

    public abstract SwerveModuleState[] getModuleStates();

    public abstract SwerveModuleState[] getModuleTargets();

    public abstract SwerveModulePosition[] getModulePositions();

    public abstract Rotation2d getRawHeading();

    public abstract SwerveSample getSwerveSample();

    public abstract double getPathXPositionError();

    public abstract double getPathYPositionError();

    public abstract double getPathThetaPositionError();

    public abstract double getPathXVelocityError();

    public abstract double getPathYVelocityError();

    public abstract double getPathThetaVelocityError();

    public abstract double getXVelocityCorrection();

    public abstract double getYVelocityCorrection();

    public abstract double getThetaCorrection();

    public abstract void setFieldCentric(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationalVelocitySetpointScalar);

    public abstract void setRobotCentric(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationalVelocitySetpointScalar);

    public abstract void setClockDrive(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationAngleDegrees);

    public abstract void setIdle();

    public abstract void setWheelAngle(double wheelAngleDegrees);

    @SuppressWarnings("unused")
    public abstract void followPath(SwerveSample sample);

    public abstract void seedFieldCentric();

    public abstract void setOperatorPerspectiveForward(Rotation2d rotation2d);

    public abstract void resetPose(Pose2d pose2d);

    public abstract void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs);

    public void update(){
        robotPose.set(getPose());
        robotSpeeds.set(getSpeeds());
        fieldSpeeds.set(getFieldSpeeds());
        moduleStates.set(getModuleStates());
        moduleTargets.set(getModuleTargets());
        modulePositions.set(getModulePositions());
        rawHeading.set(MathUtil.inputModulus(getRawHeading().getDegrees(), -180, 180));
        if (getSwerveSample() != null) {
            swerveSample.set(getSwerveSample());
        }
        if(RobotModeTriggers.autonomous().getAsBoolean()){
            pathXPositionError.set(getPathXPositionError());
            pathYPositionError.set(getPathYPositionError());
            pathThetaPositionError.set(getPathThetaPositionError() * 180 / Math.PI);
            pathXVelocityError.set(getPathXVelocityError());
            pathYVelocityError.set(getPathYVelocityError());
            pathThetaVelocityError.set(getPathThetaVelocityError() * 180 / Math.PI);
            xVelocityCorrection.set(getXVelocityCorrection());
            yVelocityCorrection.set(getYVelocityCorrection());
            thetaCorrection.set(getThetaCorrection());
        }else{
            pathXPositionError.set(0.0);
            pathYPositionError.set(0.0);
            pathThetaPositionError.set(0.0);
            pathXVelocityError.set(0.0);
            pathYVelocityError.set(0.0);
            pathThetaVelocityError.set(0.0);
            xVelocityCorrection.set(0.0);
            yVelocityCorrection.set(0.0);
            thetaCorrection.set(0.0);
        }
    }

    public abstract void updateSimState(double dt, double supplyVoltage);
}
