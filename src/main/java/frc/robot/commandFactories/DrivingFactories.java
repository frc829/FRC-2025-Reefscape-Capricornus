package frc.robot.commandFactories;

import choreo.auto.AutoFactory;
import digilib.swerve.SwerveDriveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import static edu.wpi.first.units.Units.Degrees;

public class DrivingFactories {

    public enum ReefPosition{
        LEFT, RIGHT, CENTER
    }

    private final SwerveDriveSubsystem swerve;
    private final AutoFactory autoFactory;
    private final List<Pose2d> aprilTagPoses;

    public DrivingFactories(SwerveDriveSubsystem swerve, AutoFactory autoFactory) {
        this.swerve = swerve;
        this.autoFactory = autoFactory;
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        aprilTagPoses = fieldLayout.getTags().stream().map(tag -> tag.pose.toPose2d()).toList();
    }

    public Command fieldCentricDrive(
            Supplier<Dimensionless> maxVelocityPercent,
            Supplier<Angle> headingAngle,
            Supplier<Dimensionless> maxAngularVelocityPercent) {
        SwerveDriveRequest.FieldCentric request = new SwerveDriveRequest.FieldCentric();
        return swerve.applyRequest(() -> request
                        .withVelocity(maxVelocityPercent.get())
                        .withRotationalVelocity(maxAngularVelocityPercent.get())
                        .withHeadingAngle(headingAngle.get()))
                .withName(String.format("%s: Field Centric", swerve.getName()));
    }

    public Command robotCentricDrive(
            Supplier<Dimensionless> maxVelocityPercent,
            Supplier<Angle> headingAngleRadians,
            Supplier<Dimensionless> maxAngularVelocityPercent) {
        SwerveDriveRequest.RobotCentric request = new SwerveDriveRequest.RobotCentric();
        return swerve.applyRequest(() -> request
                        .withVelocity(maxVelocityPercent.get())
                        .withRotationalVelocity(maxAngularVelocityPercent.get())
                        .withHeadingAngle(headingAngleRadians.get()))
                .withName(String.format("%s: Robot Centric", swerve.getName()));
    }

    public Command clockDrive(
            Supplier<Dimensionless> maxVelocityPercent,
            Supplier<Angle> heading,
            Supplier<Angle> rotation) {
        SwerveDriveRequest.ClockDrive request = new SwerveDriveRequest.ClockDrive();
        return swerve.applyRequest(() -> request
                        .withVelocity(maxVelocityPercent.get())
                        .withHeadingAngle(heading.get())
                        .withRotation(rotation.get()))
                .withName(String.format("%s: Clock", swerve.getName()));
    }

    public Command zeroWheels() {
        SwerveDriveRequest.PointWheels point = new SwerveDriveRequest.PointWheels();
        return swerve.applyRequest(() -> point.withDirection(Degrees.of(0.0)))
                .withName(String.format("%s: Zero Wheels", swerve.getName()));
    }

    public Command seedFieldCentric() {
        SwerveDriveRequest.SeedFieldCentric seedFieldCentric = new SwerveDriveRequest.SeedFieldCentric();
        return swerve.applyRequestOnce(() -> seedFieldCentric)
                .withName(String.format("%s: Seed Field Centric", swerve.getName()));
    }


    public Command goToTag(int tagId, ReefPosition reefPosition) {
        if(reefPosition == ReefPosition.LEFT){
            return autoFactory.trajectoryCmd(String.format("%sL", tagId));
        }else if(reefPosition == ReefPosition.RIGHT){
            return autoFactory.trajectoryCmd(String.format("%sR", tagId));
        }else{
            return autoFactory.trajectoryCmd(String.format("%s", tagId));
        }
    }

    public Trigger  isNearestTag(int tagId) {
        return new Trigger(() -> getNearestTagId() == tagId);
    }

    private int getNearestTagId() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return getNearestTagId(17, 22, swerve.getPose());
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return getNearestTagId(6, 11, swerve.getPose());
        } else {
            return -1;
        }
    }

    private int getNearestTagId(int startingTag, int endingTag, Pose2d robotLocation) {
        List<Pose2d> filteredPoses = IntStream.rangeClosed(startingTag - 1, endingTag - 1).mapToObj(tagId -> aprilTagPoses.get(tagId)).toList();
        Pose2d nearestPose = robotLocation.nearest(filteredPoses);
        return aprilTagPoses.indexOf(nearestPose) + 1;
    }
}
