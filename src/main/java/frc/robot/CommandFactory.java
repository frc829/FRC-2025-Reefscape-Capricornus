package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.SwerveDriveConstants.*;

public class CommandFactory {

    private CommandFactory() {
        // prevents instantiation
        // utility class
    }

    static class DriveCommands {

        private static boolean isClock = true;

        private static void toggleIsClock(){
            isClock = !isClock;
        }

        private static boolean getIsClock() {
            return isClock;
        }

        static Command toggleDriveState = Commands.runOnce(DriveCommands::toggleIsClock);


        static Command defaultDrive() {
            return Commands.either(
                    clockDrive(),
                    robotCentricDrive(),
                    DriveCommands::getIsClock
            ).withName("Default Drive");
        }

        static Command fieldCentricDrive() {
            SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            return commandSwerveDrive.applyRequest(() ->
                    drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(MaxAngularRate * (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis()))
                            .withRotationalDeadband(0.2 * MaxAngularRate)// Drive counterclockwise with negative X (left)
            ).withName("Field Centric Drive");
        }

        static Command robotCentricDrive() {
            SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            return commandSwerveDrive.applyRequest(() ->
                    drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(MaxAngularRate * (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis()))
                            .withRotationalDeadband(0.2 * MaxAngularRate)// Drive counterclockwise with negative X (left)
            ).withName("Robot Centric Drive");
        }

        static Command clockDrive() {
            final SwerveRequest.FieldCentricFacingAngle clockDrive = new SwerveRequest.FieldCentricFacingAngle();
            clockDrive.HeadingController = m_pathThetaController;
            return commandSwerveDrive.applyRequest(() -> {
                double x = MathUtil.applyDeadband(-driverController.getRightY(), 0.1);
                double y = MathUtil.applyDeadband(-driverController.getRightX(), 0.1);
                Rotation2d angle = new Rotation2d(x, y);
                if (x == 0 && y == 0) {
                    angle = Rotation2d.kZero;
                }
                SmartDashboard.putNumber("Request Angle (DEG)", angle.getDegrees());

                return clockDrive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                        .withTargetDirection(angle)
                        .withDeadband(MaxSpeed * 0.1);
            }).withName("Clock Drive");
        }

        static Command brake() {
            SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
            return commandSwerveDrive.applyRequest(() -> brake).withName("Brake");
        }

        static Command pointModuleWheels() {
            SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
            return commandSwerveDrive.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))).withName("point wheels");
        }

        static Command zeroWheels() {
            // TODO: Create a PointWheelsAt request
            // TODO: return the appropriate command
            return null; // TODO: remove this when done.
        }

        static Command seedFieldCentric() {
            return commandSwerveDrive.seedFieldCentric();
        }

    }

    static class SysIdCommands {

    }

}
