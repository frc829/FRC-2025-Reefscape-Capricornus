package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.mechanisms.arm.ArmRequest;
import frc.robot.mechanisms.elevator.ElevatorRequest;
import frc.robot.subsystems.CommandArm;
import frc.robot.subsystems.CommandElevator;
import frc.robot.subsystems.CommandSwerveDrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.CommandSwerveDriveConstants.*;

public class CommandFactory {
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final CommandSwerveDrive commandSwerveDrive;
    private final CommandArm commandArm;
    private final CommandElevator commandElevator;

    public CommandFactory(
            CommandXboxController driverController,
            CommandXboxController operatorController,
            CommandSwerveDrive commandSwerveDrive,
            CommandArm commandArm,
            CommandElevator commandElevator) {
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.commandSwerveDrive = commandSwerveDrive;
        this.commandArm = commandArm;
        this.commandElevator = commandElevator;
    }


    public Command defaultDrive() {
        return
                Commands.either(
                        clockDrive(),
                        robotCentricDrive(),
                        commandSwerveDrive.defaultDriveState

                );
        // return clockDrive().withName("Default Drive");
    }

    public Command fieldCentricDrive() {
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        return commandSwerveDrive.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(MaxAngularRate * (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis())).withRotationalDeadband(0.2 * MaxAngularRate)
                .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)// Drive counterclockwise with negative X (left)
        ).withName("Field Centric Drive");
    }

    public Command robotCentricDrive() {
        SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        return commandSwerveDrive.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(MaxAngularRate * (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis())).withRotationalDeadband(0.2 * MaxAngularRate)// Drive counterclockwise with negative X (left)
        ).withName("Robot Centric Drive");
    }

    public Command clockDrive() {
        final SwerveRequest.FieldCentricFacingAngle clockDrive = new SwerveRequest.FieldCentricFacingAngle();
        clockDrive.HeadingController = m_pathThetaController;
        return commandSwerveDrive.setClockDriveAngleFromPose().andThen(
                commandSwerveDrive.applyRequest(() -> {
                    double x = MathUtil.applyDeadband(-driverController.getRightY(), 0.2);
                    double y = MathUtil.applyDeadband(-driverController.getRightX(), 0.2);
                    Rotation2d rotation;
                    if (x != 0 || y != 0) {
                        rotation = new Rotation2d(x, y);
                        commandSwerveDrive.angle.mut_setMagnitude(rotation.getRadians());
                    } else {
                        rotation = Rotation2d.fromRadians(commandSwerveDrive.angle.baseUnitMagnitude());
                    }
                    SmartDashboard.putNumber("Request Angle (DEG)", rotation.getDegrees());

                    return clockDrive
                            .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                            .withDeadband(0.1 * MaxSpeed)
                            .withTargetDirection(rotation)
                            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);
                })).withName("Clock Drive");
    }

    public Command brake() {
        SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        return commandSwerveDrive.applyRequest(() -> brake).withName("Brake");
    }

    public Command pointModuleWheels() {
        SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        return commandSwerveDrive.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))).withName("point wheels");
    }

    public Command zeroWheels() {
        // TODO: Create a PointWheelsAt request
        // TODO: return the appropriate command
        return null; // TODO: remove this when done.
    }

    public Command seedFieldCentric() {
        return commandSwerveDrive.seedFieldCentric();
    }

    public Command goToReef0() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(6.177279949188232, 3.9847822189331055, Rotation2d.fromDegrees(180));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0).withName("Reef0"); // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.

    }

    public Command defaultArm() {
        ArmRequest.Hold holdRequest = new ArmRequest.Hold();
        return commandArm.applyRequest(() -> holdRequest).withName("HOLD");
    }

    public Command testCommand() {
        ArmRequest.Position position = new ArmRequest.Position()
                .withPosition(Degrees.of(10.0));
        return commandArm.applyRequest(() -> position).withName("POSITION");
    }

    public class ElevatorCommands {
        public Command defaultElevator() {
            ElevatorRequest.Hold holdRequest = new ElevatorRequest.Hold();
            return commandElevator.applyRequest(() -> holdRequest).withName("HOLD");
        }

        public Command testCommand() {
            ElevatorRequest.Position position = new ElevatorRequest.Position()
                    .withPosition(Meters.of(1));
            return commandElevator.applyRequest(() -> position).withName("Half Corbin");
        }
    }

}
