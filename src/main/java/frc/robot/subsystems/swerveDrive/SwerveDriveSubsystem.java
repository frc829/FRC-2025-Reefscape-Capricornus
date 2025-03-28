package frc.robot.subsystems.swerveDrive;

import com.ctre.phoenix6.Utils;
import digilib.cameras.Camera;
import digilib.swerve.SwerveDrive;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

public class SwerveDriveSubsystem implements Subsystem {
    private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    private final SwerveDrive swerveDrive;
    private final List<Camera> cameras;

    private double lastSimTime;
    private final Time simLoopPeriod;
    private boolean hasAppliedOperatorPerspective = false;


    public SwerveDriveSubsystem(
            SwerveDrive swerveDrive,
            Time simLoopPeriod,
            Camera... cameras) {
        this.swerveDrive = swerveDrive;
        this.simLoopPeriod = simLoopPeriod;
        this.cameras = Arrays.stream(cameras).toList();


        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command fieldCentricDrive(
            DoubleSupplier maxVelocitySetpointScalar,
            DoubleSupplier headingAngleDegrees,
            DoubleSupplier maxAngularVelocitySetpointScalar) {
        return run(() -> swerveDrive.setFieldCentric(
                maxVelocitySetpointScalar.getAsDouble(),
                headingAngleDegrees.getAsDouble(),
                maxAngularVelocitySetpointScalar.getAsDouble()))
                .withName(String.format("%s: Field Centric", getName()));
    }

    public Command robotCentricDrive(
            DoubleSupplier maxVelocitySetpointScalar,
            DoubleSupplier headingAngleDegrees,
            DoubleSupplier maxAngularVelocitySetpointScalar) {
        return run(() -> swerveDrive.setRobotCentric(
                maxVelocitySetpointScalar.getAsDouble(),
                headingAngleDegrees.getAsDouble(),
                maxAngularVelocitySetpointScalar.getAsDouble()))
                .withName(String.format("%s: Robot Centric", getName()));
    }

    public Command clockDrive(
            DoubleSupplier maxVelocitySetpointScalar,
            DoubleSupplier headingAngleDegrees,
            DoubleSupplier rotationAngleDegrees) {
        return run(() -> swerveDrive.setClockDrive(
                maxVelocitySetpointScalar.getAsDouble(),
                headingAngleDegrees.getAsDouble(),
                rotationAngleDegrees.getAsDouble()))
                .withName(String.format("%s: Clock Centric", getName()));
    }

    public Command idle() {
        return run(swerveDrive::setIdle)
                .withName(String.format("%s: Idle", getName()));
    }

    public Command zeroWheels() {
        return run(() -> swerveDrive.setWheelAngle(0.0))
                .withName(String.format("%s: Zero Wheels", getName()));
    }

    public Command seedFieldCentric() {
        return run(swerveDrive::seedFieldCentric)
                .withName(String.format("%s: Seed Field Centric", getName()));
    }

    public Command setPoseFromFront(){
        return run (() -> swerveDrive.resetPose(cameras.get(0).getRobotPose()));
    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                swerveDrive.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? RED_ALLIANCE_PERSPECTIVE_ROTATION
                                : BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                );
                hasAppliedOperatorPerspective = true;
            });
        }

        swerveDrive.update();
        cameras.forEach(camera -> {
            camera.setRobotOrientation(swerveDrive.getState().getPose());
            camera.update();
            Pose2d pose2d = camera.getRobotPose();
            double timeStampSeconds = camera.getTimeStampSeconds();
            int tagCount = camera.getTagCount();
            double poseAmbiguity = camera.getRobotPoseAmbiguity();
            Matrix<N3, N1> robotPoseStdDev = camera.getRobotPoseStdDev();
            if (Math.abs(swerveDrive.getState().getSpeeds().omegaRadiansPerSecond) > Units.degreesToRadians(360)) {
                return;
            }
            if (pose2d == null) {
                return;
            }
            if (tagCount == 0) {
                return;
            }
            if (Double.isFinite(poseAmbiguity) && poseAmbiguity >= 0.7) {
                return;
            }
            swerveDrive.addVisionMeasurement(pose2d, Utils.fpgaToCurrentTime(timeStampSeconds), robotPoseStdDev);
        });
    }

    @SuppressWarnings("resource")
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            swerveDrive.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        }).startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
