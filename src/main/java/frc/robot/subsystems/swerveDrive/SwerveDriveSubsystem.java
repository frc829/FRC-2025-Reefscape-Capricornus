package frc.robot.subsystems.swerveDrive;

import com.ctre.phoenix6.Utils;
import digilib.cameras.Camera;
import digilib.swerve.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;

public class SwerveDriveSubsystem implements Subsystem {
    private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    private final SwerveDrive swerveDrive;
    private final Camera[] cameras;

    private final List<Pose2d> aprilTagPoses;
    private double lastSimTime;
    private final Time simLoopPeriod;
    private boolean hasAppliedOperatorPerspective = false;


    public SwerveDriveSubsystem(
            SwerveDrive swerveDrive,
            Time simLoopPeriod,
            AprilTagFieldLayout aprilTagFieldLayout,
            Camera... cameras) {
        this.swerveDrive = swerveDrive;
        this.simLoopPeriod = simLoopPeriod;
        this.cameras = cameras;

        aprilTagPoses = aprilTagFieldLayout.getTags().stream().map(tag -> tag.pose.toPose2d()).toList();

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

    public Command pointSteer(DoubleSupplier angleDegrees) {
        return run(() -> swerveDrive.setSteerAngle(angleDegrees.getAsDouble()));
    }

    public Command brake() {
        return run(() -> swerveDrive.setBrake())
                .withName(String.format("%s: Brake", getName()));
    }

    public Command idle() {
        return run(() -> swerveDrive.setIdle())
                .withName(String.format("%s: Idle", getName()));
    }

    public Command zeroWheels() {
        return run(() -> swerveDrive.setWheelAngle(0.0))
                .withName(String.format("%s: Zero Wheels", getName()));
    }

    public Command seedFieldCentric() {
        return run(() -> swerveDrive.seedFieldCentric())
                .withName(String.format("%s: Seed Field Centric", getName()));
    }

    public Command setFieldFromCamera() {
        return run(() -> {
            if (cameras[0].getState().getRobotPose().isPresent()) {
                swerveDrive.resetPose(cameras[0].getState().getRobotPose().get().estimatedPose.toPose2d());
            }
        });
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
        cameras[0].update();
        if (RobotBase.isReal()) {
            if (cameras[0].getState().getRobotPose().isPresent() && cameras[0].getState().getRobotPoseStdDev().isPresent()) {
                var cameraPose = cameras[0].getState().getRobotPose().get().estimatedPose.toPose2d();
                var timeStampSeconds = cameras[0].getState().getRobotPose().get().timestampSeconds;
                var robotPose = swerveDrive.getState().getPose();
                timeStampSeconds = Utils.fpgaToCurrentTime(timeStampSeconds);
                if (cameraPose.getTranslation().getDistance(robotPose.getTranslation()) < 0.05) {
                    swerveDrive.addVisionMeasurement(
                            cameraPose,
                            timeStampSeconds,
                            MatBuilder.fill(Nat.N3(), Nat.N1(), 0.01, 0.01, Double.MAX_VALUE));
                    // camera.getState().getRobotPoseStdDev().get());
                }
            }
        }

    }

    private int getNearestTagId(int startingTag, int endingTag, Pose2d robotLocation) {
        List<Pose2d> filteredPoses = IntStream.rangeClosed(startingTag - 1, endingTag - 1).mapToObj(tagId -> aprilTagPoses.get(tagId)).toList();
        Pose2d nearestPose = robotLocation.nearest(filteredPoses);
        return aprilTagPoses.indexOf(nearestPose) + 1;
    }

    private int getNearestTagId() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return getNearestTagId(17, 22, swerveDrive.getState().getPose());
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return getNearestTagId(6, 11, swerveDrive.getState().getPose());
        } else {
            return -1;
        }
    }

    public Trigger isNearestTag(int tagId) {
        return new Trigger(() -> getNearestTagId() == tagId);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            swerveDrive.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            for (var camera : cameras) {
                camera.updateSimState(swerveDrive.getState().getPose());
            }
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }


}
