package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        static Command clockDrive(){
            final SwerveRequest.FieldCentricFacingAngle clockDrive = new SwerveRequest.FieldCentricFacingAngle();
            return drivetrain.applyRequest(() -> {
                double x = MathUtil.applyDeadband(-driverController.getRightY(), 0.1);
                double y = MathUtil.applyDeadband(-driverController.getRightX(),0.1);
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


        static Command brake(){
            SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
            return drivetrain.applyRequest(() -> brake).withName("Brake");
        }

        static Command pointModuleWheels(){
            SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
            return drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))).withName("point wheels");
        }

        static Command robotCentricForward(){
            SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
            return drivetrain.applyRequest(()-> forwardStraight.withVelocityX(.5).withVelocityY(0));
        }

        static  Command robotCentricReverse(){
            SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
            return drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-.5).withVelocityY(0));
        }

    }

    static class SysIdCommands{

    }

}
