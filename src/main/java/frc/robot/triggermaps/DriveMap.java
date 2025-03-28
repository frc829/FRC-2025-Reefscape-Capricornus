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
        bindSlowFieldCentricDrive();
        bindRobotCentricDrive();
        bindSlowRobotCentricDrive();
        bindClockDrive();
        bindSlowClockDrive();

        bindZeroWheel();
        bindSeedFieldCentric();

        bindSetFieldFromCamera();
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

    private double getSlowMaxVelocitySetpointSquaredScalar() {
        return 0.1 * getMaxVelocitySetpointSquaredScalar();
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
        (notClockDrive.or(rotationalVelocity)).and(driver.y().negate()).and(driver.a().negate())
                .whileTrue(swerveDriveSubsystem.fieldCentricDrive(
                        this::getMaxVelocitySetpointSquaredScalar,
                        this::getHeadingDegrees,
                        this::getMaxAngularVelocitySetpointScalar));
    }

    private void bindSlowFieldCentricDrive() {
        Trigger leftStickTrigger = new Trigger(() -> getMaxVelocitySetpointSquaredScalar() != 0.0);
        Trigger rightStickTrigger = new Trigger(() -> getRightStickValue() != 0.0);
        Trigger notClockDrive = leftStickTrigger.and(rightStickTrigger.negate());
        Trigger rotationalVelocity = new Trigger(() -> getMaxAngularVelocitySetpointScalar() != 0.0);
        (notClockDrive.or(rotationalVelocity)).and(driver.y().negate()).and(driver.a())
                .whileTrue(swerveDriveSubsystem.fieldCentricDrive(
                        this::getSlowMaxVelocitySetpointSquaredScalar,
                        this::getHeadingDegrees,
                        this::getMaxAngularVelocitySetpointScalar).withName("Slow Field Centric"));
    }

    private void bindRobotCentricDrive() {
        Trigger leftStickTrigger = new Trigger(() -> getMaxVelocitySetpointSquaredScalar() != 0.0);
        Trigger rightStickTrigger = new Trigger(() -> getRightStickValue() != 0.0);
        Trigger notClockDrive = leftStickTrigger.and(rightStickTrigger.negate());
        Trigger rotationalVelocity = new Trigger(() -> getMaxAngularVelocitySetpointScalar() != 0.0);
        (notClockDrive.or(rotationalVelocity)).and(driver.y()).and(driver.a().negate())
                .whileTrue(swerveDriveSubsystem.robotCentricDrive(
                        this::getMaxVelocitySetpointSquaredScalar,
                        this::getHeadingDegrees,
                        this::getMaxAngularVelocitySetpointScalar));
    }

    private void bindSlowRobotCentricDrive() {
        Trigger leftStickTrigger = new Trigger(() -> getMaxVelocitySetpointSquaredScalar() != 0.0);
        Trigger rightStickTrigger = new Trigger(() -> getRightStickValue() != 0.0);
        Trigger notClockDrive = leftStickTrigger.and(rightStickTrigger.negate());
        Trigger rotationalVelocity = new Trigger(() -> getMaxAngularVelocitySetpointScalar() != 0.0);
        driver.a().and((notClockDrive.or(rotationalVelocity)).and(driver.y()).and(driver.a().negate())
                .whileTrue(swerveDriveSubsystem.robotCentricDrive(
                        this::getMaxVelocitySetpointSquaredScalar,
                        this::getHeadingDegrees,
                        this::getMaxAngularVelocitySetpointScalar).withName("Slow Robot Centric")));
    }

    private void bindClockDrive() {
        Trigger rightStickTrigger = new Trigger(() -> getRightStickValue() != 0.0);
        Trigger rotationalVelocity = new Trigger(() -> getMaxAngularVelocitySetpointScalar() != 0.0);
        rightStickTrigger.and(rotationalVelocity.negate()).and(driver.a().negate())
                .whileTrue(swerveDriveSubsystem.clockDrive(
                        this::getMaxVelocitySetpointSquaredScalar,
                        this::getHeadingDegrees,
                        this::getRotationDegrees));


    }

    private void bindSlowClockDrive() {
        Trigger rightStickTrigger = new Trigger(() -> getRightStickValue() != 0.0);
        Trigger rotationalVelocity = new Trigger(() -> getMaxAngularVelocitySetpointScalar() != 0.0);
        rightStickTrigger.and(rotationalVelocity.negate()).and(driver.a())
                .whileTrue(swerveDriveSubsystem.clockDrive(
                        this::getSlowMaxVelocitySetpointSquaredScalar,
                        this::getHeadingDegrees,
                        this::getRotationDegrees).withName("Slow Clock Drive"));
    }

    private void bindZeroWheel() {
        driver.back().whileTrue(swerveDriveSubsystem.zeroWheels());
    }

    private void bindSeedFieldCentric() {
        driver.start().onTrue(swerveDriveSubsystem.seedFieldCentric());
    }

    private void bindSetFieldFromCamera() {
        driver.b().whileTrue(swerveDriveSubsystem.setPoseFromFront());
    }
}
