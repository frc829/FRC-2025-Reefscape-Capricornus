package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.driverController;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.constants.SwerveDriveConstants.MaxAngularRate;
import static frc.robot.constants.SwerveDriveConstants.MaxSpeed;

public class CommandFactory {

    private CommandFactory() {
        // prevents instantiation
        // utility class
    }

    static class DriveCommands{
        static Command fieldCentricDrive(){
            SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            return drivetrain.applyRequest(() ->
                    drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(MaxAngularRate * (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis()))
                            .withRotationalDeadband(0.2 * MaxAngularRate)// Drive counterclockwise with negative X (left)
            ).withName("Field Centric Drive");
        }

        static Command brake(){
            SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
            return drivetrain.applyRequest(() -> brake).withName("Brake");
        }
    }

    static class SysIdCommands{

    }

}
