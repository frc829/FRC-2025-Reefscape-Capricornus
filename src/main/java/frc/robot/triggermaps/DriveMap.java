package frc.robot.triggermaps;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;

import static frc.robot.triggermaps.DriveMap.ReefPosition.*;
import static java.lang.Math.pow;
import static java.lang.Math.toDegrees;

public class DriveMap {

    public enum ReefPosition{
        LEFT,
        CENTER,
        RIGHT
    }


    private final CommandXboxController driver;
    private final double deadband;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final AutoFactory autoFactory;

    public DriveMap(
            CommandXboxController driver,
            double deadband,
            SwerveDriveSubsystem swerveDriveSubsystem,
            AutoFactory autoFactory) {
        this.driver = driver;
        this.deadband = deadband;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.autoFactory = autoFactory;

        bindFieldCentricDrive();
        bindRobotCentricDrive();
        bindClockDrive();

        bindZeroWheel();
        bindSeedFieldCentric();

        bindSetFieldFromCamera();

        bindGoToNearestLeftReef();
        bindGoToNearestRightReef();
        bindGoToNearestAlgae();
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

    private void bindGoToNearestLeftReef() {
        driver.povLeft().and(swerveDriveSubsystem.isNearestTag(17).or(swerveDriveSubsystem.isNearestTag(8)))
                .whileTrue(goToTag(17, LEFT));
        driver.povLeft().and(swerveDriveSubsystem.isNearestTag(18).or(swerveDriveSubsystem.isNearestTag(7)))
                .whileTrue(goToTag(18, LEFT));
        driver.povLeft().and(swerveDriveSubsystem.isNearestTag(19).or(swerveDriveSubsystem.isNearestTag(6)))
                .whileTrue(goToTag(19, LEFT));
        driver.povLeft().and(swerveDriveSubsystem.isNearestTag(20).or(swerveDriveSubsystem.isNearestTag(11)))
                .whileTrue(goToTag(20, LEFT));
        driver.povLeft().and(swerveDriveSubsystem.isNearestTag(21).or(swerveDriveSubsystem.isNearestTag(10)))
                .whileTrue(goToTag(21, LEFT));
        driver.povLeft().and(swerveDriveSubsystem.isNearestTag(22).or(swerveDriveSubsystem.isNearestTag(9)))
                .whileTrue(goToTag(22, LEFT));
    }

    private void bindGoToNearestRightReef() {
        driver.povRight().and(swerveDriveSubsystem.isNearestTag(17).or(swerveDriveSubsystem.isNearestTag(8)))
                .whileTrue(goToTag(17, RIGHT));
        driver.povRight().and(swerveDriveSubsystem.isNearestTag(18).or(swerveDriveSubsystem.isNearestTag(7)))
                .whileTrue(goToTag(18, RIGHT));
        driver.povRight().and(swerveDriveSubsystem.isNearestTag(19).or(swerveDriveSubsystem.isNearestTag(6)))
                .whileTrue(goToTag(19, RIGHT));
        driver.povRight().and(swerveDriveSubsystem.isNearestTag(20).or(swerveDriveSubsystem.isNearestTag(11)))
                .whileTrue(goToTag(20, RIGHT));
        driver.povRight().and(swerveDriveSubsystem.isNearestTag(21).or(swerveDriveSubsystem.isNearestTag(10)))
                .whileTrue(goToTag(21, RIGHT));
        driver.povRight().and(swerveDriveSubsystem.isNearestTag(22).or(swerveDriveSubsystem.isNearestTag(9)))
                .whileTrue(goToTag(22, RIGHT));
    }

    private void bindGoToNearestAlgae() {
        driver.povUp().and(swerveDriveSubsystem.isNearestTag(17).or(swerveDriveSubsystem.isNearestTag(8)))
                .whileTrue(goToTag(17, CENTER));
        driver.povUp().and(swerveDriveSubsystem.isNearestTag(18).or(swerveDriveSubsystem.isNearestTag(7)))
                .whileTrue(goToTag(18, CENTER));
        driver.povUp().and(swerveDriveSubsystem.isNearestTag(19).or(swerveDriveSubsystem.isNearestTag(6)))
                .whileTrue(goToTag(19, CENTER));
        driver.povUp().and(swerveDriveSubsystem.isNearestTag(20).or(swerveDriveSubsystem.isNearestTag(11)))
                .whileTrue(goToTag(20, CENTER));
        driver.povUp().and(swerveDriveSubsystem.isNearestTag(21).or(swerveDriveSubsystem.isNearestTag(10)))
                .whileTrue(goToTag(21, CENTER));
        driver.povUp().and(swerveDriveSubsystem.isNearestTag(22).or(swerveDriveSubsystem.isNearestTag(9)))
                .whileTrue(goToTag(22, CENTER));
    }



    public Command goToTag(int tagId, ReefPosition reefPosition) {
        if (reefPosition == ReefPosition.LEFT) {
            return autoFactory.trajectoryCmd(String.format("%sL", tagId));
        } else if (reefPosition == ReefPosition.RIGHT) {
            return autoFactory.trajectoryCmd(String.format("%sR", tagId));
        } else {
            return autoFactory.trajectoryCmd(String.format("%s", tagId));
        }
    }


}
