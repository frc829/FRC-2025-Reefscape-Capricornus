package frc.robot.commandFactories;

import digilib.swerve.SwerveDriveRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;

public class DrivingFactories {

    private final SwerveDriveSubsystem swerve;

    public DrivingFactories(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
    }

    public Command fieldCentricDrive(
            Supplier<Dimensionless> maxVelocityPercent,
            Supplier<Dimensionless> maxAngularVelocityPercent,
            Supplier<Angle> headingAngleRadians) {
        SwerveDriveRequest.FieldCentric request = new SwerveDriveRequest.FieldCentric();
        return swerve.applyRequest(() -> request
                        .withVelocity(maxVelocityPercent.get())
                        .withRotationalVelocity(maxAngularVelocityPercent.get())
                        .withHeadingAngle(headingAngleRadians.get()))
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

    public Command brake() {
        SwerveDriveRequest.Brake brake = new SwerveDriveRequest.Brake();
        return swerve.applyRequest(() -> brake)
                .withName(String.format("%s: Brake", swerve.getName()));
    }

    public Command pointModuleWheels(Supplier<Angle> angle) {
        SwerveDriveRequest.PointWheels point = new SwerveDriveRequest.PointWheels();
        return swerve.applyRequest(() -> point.withDirection(angle.get()))
                .withName(String.format("%s: Point Wheels", swerve.getName()));
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
}
