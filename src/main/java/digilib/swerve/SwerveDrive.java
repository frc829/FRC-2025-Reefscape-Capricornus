package digilib.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface SwerveDrive {

    SwerveDriveState getState();

    void setFieldCentric(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationalVelocitySetpointScalar);

    void setRobotCentric(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationalVelocitySetpointScalar);

    void setClockDrive(
            double translationVelocitySetpointScalar,
            double headingAngleDegrees,
            double rotationAngleDegrees);

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

    void updateSimState(double dt, double supplyVoltage);

}
