package frc.robot.triggermaps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;

import static java.lang.Math.pow;
import static java.lang.Math.toDegrees;

public class DriveMap {
    private final CommandXboxController driver;
    private final double deadband;
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    public DriveMap(
            CommandXboxController driver,
            double deadband,
            SwerveDriveSubsystem swerveDriveSubsystem) {
        this.driver = driver;
        this.deadband = deadband;
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        bindFieldCentricDrive();
        bindRobotCentricDrive();
        bindClockDrive();

        bindZeroWheel();
        bindSeedFieldCentric();

        bindSetFieldFromCamera();

        // bindGoToNearestLeftReef();
        // bindGoToNearestRightReef();
        // bindGoToNearestAlgae();
    }

    private double getMaxVelocitySetpointScalar() {
        double x = -MathUtil.applyDeadband(driver.getLeftY(), deadband);
        double y = -MathUtil.applyDeadband(driver.getLeftX(), deadband);
        double velocity = Math.hypot(x, y);
        return Math.min(velocity, 1);
    }

    private double getMaxVelocitySetpointSquaredScalar() {
        return pow(getMaxVelocitySetpointScalar(), 2);
    }

    private double getMaxAngularVelocitySetpointScalar() {
        double leftTrigger = MathUtil.applyDeadband(driver.getLeftTriggerAxis(), deadband);
        double rightTrigger = MathUtil.applyDeadband(driver.getRightTriggerAxis(), deadband);
        return leftTrigger - rightTrigger;
    }

    private double getHeadingDegrees() {
        double x = -MathUtil.applyDeadband(driver.getLeftY(), deadband);
        double y = -MathUtil.applyDeadband(driver.getLeftX(), deadband);
        return toDegrees(Math.atan2(y, x));
    }

    private double getRotationDegrees() {
        double x = -MathUtil.applyDeadband(driver.getRightY(), deadband);
        double y = -MathUtil.applyDeadband(driver.getRightX(), deadband);
        return toDegrees(Math.atan2(y, x));
    }

    private double getRightStickValue() {
        double x = -MathUtil.applyDeadband(driver.getRightY(), deadband);
        double y = -MathUtil.applyDeadband(driver.getRightX(), deadband);
        return Math.hypot(x, y);
    }

    private void bindFieldCentricDrive() {
        Trigger leftStickTrigger = new Trigger(() -> getMaxVelocitySetpointSquaredScalar() != 0.0);
        Trigger rightStickTrigger = new Trigger(() -> getRightStickValue() != 0.0);
        Trigger notClockDrive = leftStickTrigger.and(rightStickTrigger.negate());
        Trigger rotationalVelocity = new Trigger(() -> getMaxAngularVelocitySetpointScalar() != 0.0);
        (notClockDrive.or(rotationalVelocity)).and(driver.y().negate())
                .whileTrue(swerveDriveSubsystem.fieldCentricDrive(
                        this::getMaxVelocitySetpointSquaredScalar,
                        this::getHeadingDegrees,
                        this::getMaxAngularVelocitySetpointScalar));
    }

    private void bindRobotCentricDrive() {
        Trigger leftStickTrigger = new Trigger(() -> getMaxVelocitySetpointSquaredScalar() != 0.0);
        Trigger rightStickTrigger = new Trigger(() -> getRightStickValue() != 0.0);
        Trigger notClockDrive = leftStickTrigger.and(rightStickTrigger.negate());
        Trigger rotationalVelocity = new Trigger(() -> getMaxAngularVelocitySetpointScalar() != 0.0);
        (notClockDrive.or(rotationalVelocity)).and(driver.y())
                .whileTrue(swerveDriveSubsystem.robotCentricDrive(
                        this::getMaxVelocitySetpointSquaredScalar,
                        this::getHeadingDegrees,
                        this::getMaxAngularVelocitySetpointScalar));
    }

    private void bindClockDrive() {
        Trigger rightStickTrigger = new Trigger(() -> getRightStickValue() != 0.0);
        Trigger rotationalVelocity = new Trigger(() -> getMaxAngularVelocitySetpointScalar() != 0.0);
        rightStickTrigger.and(rotationalVelocity.negate())
                .whileTrue(swerveDriveSubsystem.clockDrive(
                        this::getMaxVelocitySetpointSquaredScalar,
                        this::getHeadingDegrees,
                        this::getRotationDegrees));


    }

    private void bindZeroWheel() {
        driver.back().whileTrue(swerveDriveSubsystem.zeroWheels());
    }

    private void bindSeedFieldCentric() {
        driver.start().onTrue(swerveDriveSubsystem.seedFieldCentric());
    }

    private void bindSetFieldFromCamera() {
        driver.b().whileTrue(swerveDriveSubsystem.setFieldFromCamera());
    }

    // private int getNearestTagId(int startingTag, int endingTag, Pose2d robotLocation) {
    //     List<Pose2d> filteredPoses = IntStream.rangeClosed(startingTag - 1, endingTag - 1).mapToObj(tagId -> aprilTagPoses.get(tagId)).toList();
    //     Pose2d nearestPose = robotLocation.nearest(filteredPoses);
    //     return aprilTagPoses.indexOf(nearestPose) + 1;
    // }
    //
    // private int getNearestTagId() {
    //     if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    //         return getNearestTagId(17, 22, swerveDrive.getState().getPose());
    //     } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    //         return getNearestTagId(6, 11, swerveDrive.getState().getPose());
    //     } else {
    //         return -1;
    //     }
    // }
    //
    // public Trigger isNearestTag(int tagId) {
    //     return new Trigger(() -> getNearestTagId() == tagId);
    // }
    //
    // public Command goToTag(int tagId, Drive.ReefPosition reefPosition) {
    //     if (reefPosition == Drive.ReefPosition.LEFT) {
    //         return autoFactory.trajectoryCmd(String.format("%sL", tagId));
    //     } else if (reefPosition == Drive.ReefPosition.RIGHT) {
    //         return autoFactory.trajectoryCmd(String.format("%sR", tagId));
    //     } else {
    //         return autoFactory.trajectoryCmd(String.format("%s", tagId));
    //     }
    // }

    // private void bindGoToNearestLeftReef() {
    //     driver.povLeft().and(driving.isNearestTag(17).or(driving.isNearestTag(8)))
    //             .whileTrue(driving.goToTag(17, LEFT));
    //     driver.povLeft().and(driving.isNearestTag(18).or(driving.isNearestTag(7)))
    //             .whileTrue(driving.goToTag(18, LEFT));
    //     driver.povLeft().and(driving.isNearestTag(19).or(driving.isNearestTag(6)))
    //             .whileTrue(driving.goToTag(19, LEFT));
    //     driver.povLeft().and(driving.isNearestTag(20).or(driving.isNearestTag(11)))
    //             .whileTrue(driving.goToTag(20, LEFT));
    //     driver.povLeft().and(driving.isNearestTag(21).or(driving.isNearestTag(10)))
    //             .whileTrue(driving.goToTag(21, LEFT));
    //     driver.povLeft().and(driving.isNearestTag(22).or(driving.isNearestTag(9)))
    //             .whileTrue(driving.goToTag(22, LEFT));
    // }
    //
    // private void bindGoToNearestRightReef() {
    //     driver.povRight().and(driving.isNearestTag(17).or(driving.isNearestTag(8)))
    //             .whileTrue(driving.goToTag(17, RIGHT));
    //     driver.povRight().and(driving.isNearestTag(18).or(driving.isNearestTag(7)))
    //             .whileTrue(driving.goToTag(18, RIGHT));
    //     driver.povRight().and(driving.isNearestTag(19).or(driving.isNearestTag(6)))
    //             .whileTrue(driving.goToTag(19, RIGHT));
    //     driver.povRight().and(driving.isNearestTag(20).or(driving.isNearestTag(11)))
    //             .whileTrue(driving.goToTag(20, RIGHT));
    //     driver.povRight().and(driving.isNearestTag(21).or(driving.isNearestTag(10)))
    //             .whileTrue(driving.goToTag(21, RIGHT));
    //     driver.povRight().and(driving.isNearestTag(22).or(driving.isNearestTag(9)))
    //             .whileTrue(driving.goToTag(22, RIGHT));
    // }
    //
    // private void bindGoToNearestAlgae() {
    //     driver.povUp().and(driving.isNearestTag(17).or(driving.isNearestTag(8)))
    //             .whileTrue(driving.goToTag(17, CENTER));
    //     driver.povUp().and(driving.isNearestTag(18).or(driving.isNearestTag(7)))
    //             .whileTrue(driving.goToTag(18, CENTER));
    //     driver.povUp().and(driving.isNearestTag(19).or(driving.isNearestTag(6)))
    //             .whileTrue(driving.goToTag(19, CENTER));
    //     driver.povUp().and(driving.isNearestTag(20).or(driving.isNearestTag(11)))
    //             .whileTrue(driving.goToTag(20, CENTER));
    //     driver.povUp().and(driving.isNearestTag(21).or(driving.isNearestTag(10)))
    //             .whileTrue(driving.goToTag(21, CENTER));
    //     driver.povUp().and(driving.isNearestTag(22).or(driving.isNearestTag(9)))
    //             .whileTrue(driving.goToTag(22, CENTER));
    // }
}