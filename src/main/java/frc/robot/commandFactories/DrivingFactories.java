package frc.robot.commandFactories;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import digilib.swerve.SwerveDriveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class DrivingFactories {

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

    public AutoRoutine goToNearestTag() {
        Map<Integer, Command> trajCommands = new HashMap<>();
        AutoRoutine routine = autoFactory.newRoutine("Nearest Tag");
        trajCommands.put(1, idle());
        trajCommands.put(2, idle());
        trajCommands.put(3, idle());
        trajCommands.put(4, idle());
        trajCommands.put(5, idle());
        trajCommands.put(6, idle());
        trajCommands.put(7, idle());
        trajCommands.put(8, idle());
        trajCommands.put(9, idle());
        trajCommands.put(10, idle());
        trajCommands.put(11, idle());
        trajCommands.put(12, idle());
        trajCommands.put(13, idle());
        trajCommands.put(14, idle());
        trajCommands.put(15, idle());
        trajCommands.put(16, idle());
        trajCommands.put(17, routine.trajectory("17").cmd());
        trajCommands.put(18, routine.trajectory("18").cmd());
        trajCommands.put(19, idle());
        trajCommands.put(20, idle());
        trajCommands.put(21, idle());
        trajCommands.put(22, idle());
        trajCommands.put(-1, idle());
        Command[] trajectoryCmd = new Command[]{idle()};
        routine.active().onTrue(
                sequence(
                        runOnce(() -> trajectoryCmd[0] = trajCommands.get(getNearestTagId())),
                        runOnce(() -> SmartDashboard.putNumber("Nearest", getNearestTagId())),
                        trajectoryCmd[0]));
        return routine;
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
