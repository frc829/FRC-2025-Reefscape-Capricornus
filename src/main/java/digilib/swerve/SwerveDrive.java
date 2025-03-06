package digilib.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface SwerveDrive {

    double getMaxVelocityMPS();

    double getMaxAngularVelocityRPS();

    SwerveDriveState getState();

    void setFieldCentric(
            double translationalVelocityMPS,
            double headingAngleDegrees,
            double rotationalVelocityDPS);

    void setRobotCentric(
            double translationalVelocityMPS,
            double headingAngleDegrees,
            double rotationalVelocityDPS);

    void setClockDrive(
            double translationalVelocityMPS,
            double headingAngleDegrees,
            double rotationAngleDegrees);

    void setBrake();

    void setIdle();

    void setWheelAngle(double wheelAngleDegrees);

    void followPath(SwerveSample sample);

    void seedFieldCentric();

    void setOperatorPerspectiveForward(Rotation2d rotation2d);

    void resetPose(Pose2d pose2d);

    void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs);

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);

}
