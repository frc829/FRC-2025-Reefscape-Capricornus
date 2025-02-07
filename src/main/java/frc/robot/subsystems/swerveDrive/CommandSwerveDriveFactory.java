package frc.robot.subsystems.swerveDrive;

import digilib.swerve.SwerveDriveRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;

public class CommandSwerveDriveFactory {

    private final CommandSwerveDrive commandSwerveDrive;

    public CommandSwerveDriveFactory(CommandSwerveDrive commandSwerveDrive) {
        this.commandSwerveDrive = commandSwerveDrive;
        commandSwerveDrive.setDefaultCommand(idle());
    }

    public Command fieldCentricDrive(
            Supplier<Dimensionless> maxVelocityPercent,
            Supplier<Dimensionless> maxAngularVelocityPercent,
            Supplier<Angle> headingAngleRadians) {
        SwerveDriveRequest.FieldCentric request = new SwerveDriveRequest.FieldCentric();
        return commandSwerveDrive.applyRequest(() -> request
                        .withVelocity(maxVelocityPercent.get())
                        .withRotationalVelocity(maxAngularVelocityPercent.get())
                        .withHeadingAngle(headingAngleRadians.get()))
                .withName("Field Centric Drive");
    }

    public Command robotCentricDrive(
            Supplier<Dimensionless> maxVelocityPercent,
            Supplier<Dimensionless> maxAngularVelocityPercent,
            Supplier<Angle> headingAngleRadians) {
        SwerveDriveRequest.RobotCentric request = new SwerveDriveRequest.RobotCentric();
        return commandSwerveDrive.applyRequest(() -> request
                        .withVelocity(maxVelocityPercent.get())
                        .withRotationalVelocity(maxAngularVelocityPercent.get())
                        .withHeadingAngle(headingAngleRadians.get()))
                .withName("Robot Centric Drive");
    }

    public Command clockDrive(
            Supplier<Dimensionless> maxVelocityPercent,
            Supplier<Angle> heading,
            Supplier<Angle> rotation) {
        SwerveDriveRequest.ClockDrive request = new SwerveDriveRequest.ClockDrive();
        return commandSwerveDrive.applyRequest(() -> request
                        .withVelocity(maxVelocityPercent.get())
                        .withHeadingAngle(heading.get())
                        .withRotation(rotation.get()))
                .withName("Clock Drive");
    }

    public Command brake() {
        SwerveDriveRequest.Brake brake = new SwerveDriveRequest.Brake();
        return commandSwerveDrive.applyRequest(() -> brake).withName("Brake");
    }

    public Command pointModuleWheels(Supplier<Angle> angle) {
        SwerveDriveRequest.PointWheels point = new SwerveDriveRequest.PointWheels();
        return commandSwerveDrive.applyRequest(() -> point.withDirection(angle.get()))
                .withName("Point wheels");
    }

    public Command zeroWheels() {
        SwerveDriveRequest.PointWheels point = new SwerveDriveRequest.PointWheels();
        return commandSwerveDrive.applyRequest(() -> point.withDirection(Degrees.of(0.0)))
                .withName("Zero Wheels");
    }

    public Command seedFieldCentric() {
        SwerveDriveRequest.SeedFieldCentric seedFieldCentric = new SwerveDriveRequest.SeedFieldCentric();
        return commandSwerveDrive.applyRequestOnce(() -> seedFieldCentric)
                .withName("Seed Field Centric");
    }

    public Command idle(){
        SwerveDriveRequest.Idle idle = new SwerveDriveRequest.Idle();
        return commandSwerveDrive.applyRequest(() -> idle).withName("Idle");
    }

//    public Command goToReef0() {
//        // Since we are using a holonomic drivetrain, the rotation component of this pose
//        // represents the goal holonomic rotation
//        Pose2d targetPose = new Pose2d(6.177279949188232, 3.9847822189331055, Rotation2d.fromDegrees(180));
//
//        // Create the constraints to use while pathfinding
//        PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
//
//        // Since AutoBuilder is configured, we can use it to build pathfinding commands
//        return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0).withName("Reef0"); // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
//
//    }
}
