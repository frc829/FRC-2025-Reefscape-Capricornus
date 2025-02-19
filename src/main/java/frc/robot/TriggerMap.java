package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandFactories.*;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.Joystick.AxisType.*;
import static edu.wpi.first.wpilibj.XboxController.Axis.*;

public class TriggerMap {
    private static final double deadband = 0.1;
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandJoystick climb = new CommandJoystick(2);
    private final CommandXboxController backup = new CommandXboxController(3);

    private final MutDimensionless maxVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxRotationalVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxManualWristVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxManualElevatorVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxManualArmVelocityPercent = Value.mutable(0.0);
    private final MutAngle heading = Radians.mutable(0.0);
    private final MutAngle rotation = Radians.mutable(0.0);

    private final DrivingFactories driving;
    private final PickupFactories pickup;
    private final ScoringFactories scoring;
    private final ManualFactories manual;

    public TriggerMap(DrivingFactories driving,
                      PickupFactories pickup,
                      ScoringFactories scoring,
                      ManualFactories manual) {
        this.driving = driving;
        this.pickup = pickup;
        this.scoring = scoring;
        this.manual = manual;

        bindClockDrive();
        bindRobotCentricDrive();
        bindAlgaeFloorPickup();
        bindAlgaeL2Pickup();
        bindAlgaeL3Pickup();
        bindCoralFloorPickup();
        bindCoralStationPickup();
        bindBargeScore();
        bindProcessorScore();
        bindL1Score();
        bindL2Score();
        bindL3Score();
        bindL4Score();
        bindClimbScore();
        bindManualWristToggle();
        bindManualWrist();
        bindManualElevator();
        bindManualArm();
        bindManualAlgaeClawToggle();
        bindManualCoralClawToggle();
        bindManualCoralIn();
        bindManualCoralOut();
        bindManualAlgaeIn();
        bindManualAlgaeOut();

    }

    private void bindClockDrive() {
        new Trigger(() -> getMaxVelocityPercentValue() != 0.0)
                .and(() -> {
                    double x = MathUtil.applyDeadband(driver.getRightX(), deadband);
                    double y = MathUtil.applyDeadband(driver.getRightY(), deadband);
                    return Math.hypot(x, y) != 0.0;
                }).whileTrue(driving.clockDrive(
                        this::getMaxVelocityPercent,
                        this::getHeading,
                        this::getRotation));
    }

    private void bindRobotCentricDrive() {
        new Trigger(() -> getMaxVelocityPercentValue() != 0.0)
                .and(() -> {
                    double x = MathUtil.applyDeadband(driver.getRightX(), deadband);
                    double y = MathUtil.applyDeadband(driver.getRightY(), deadband);
                    return Math.hypot(x, y) == 0.0;
                }).whileTrue(driving.robotCentricDrive(
                        this::getMaxVelocityPercent,
                        this::getHeading,
                        this::getMaxRotationalVelocityPercent));
    }

    private void bindAlgaeFloorPickup() {
        operator.axisMagnitudeGreaterThan(kLeftTrigger.value, deadband)
                .whileTrue(pickup.algaeFloor())
                .onFalse(pickup.holdAfterAlgae());
    }

    private void bindAlgaeL2Pickup() {
        operator.rightStick()
                .whileTrue(pickup.algaeL2())
                .onFalse(pickup.holdAfterAlgae());
    }

    private void bindAlgaeL3Pickup() {
        operator.leftStick()
                .whileTrue(pickup.algaeL3())
                .onFalse(pickup.holdAfterAlgae());
    }

    private void bindCoralFloorPickup() {
        operator.rightBumper()
                .whileTrue(pickup.coralFloor())
                .onFalse(pickup.holdAfterCoral());
    }

    private void bindCoralStationPickup() {
        operator.povUp()
                .whileTrue(pickup.coralStation())
                .onFalse(pickup.holdAfterCoral());
    }

    private void bindBargeScore() {
        operator.povLeft();
    }

    private void bindProcessorScore() {
        operator.povDown();
    }

    private void bindL1Score() {
        driver.a();
    }

    private void bindL2Score() {
        driver.x();
    }

    private void bindL3Score() {
        driver.b();
    }

    private void bindL4Score() {
        driver.y();
    }

    private void bindClimbScore() {
        climb.axisMagnitudeGreaterThan(kY.value, deadband);
    }

    private void bindManualWristToggle() {
        backup.y().onTrue(manual.manualWristToggle());
    }

    private void bindManualWrist() {
        new Trigger(() -> getMaxWristVelocityPercentValue() != 0.0)
                .whileTrue(manual.manualWrist(this::getMaxWristVelocityPercent));
    }

    private void bindManualElevator() {
        new Trigger(() -> getMaxElevatorVelocityPercentValue() != 0.0)
                .whileTrue(manual.manualElevator(this::getMaxElevatorVelocityPercent));
    }

    private void bindManualArm() {
        new Trigger(() -> getMaxArmVelocityPercentValue() != 0.0)
                .whileTrue(manual.manualArm(this::getMaxArmVelocityPercent));
    }

    private void bindManualAlgaeClawToggle() {
        backup.leftBumper()
                .onTrue(manual.manualAlgaeClawToggle());
    }

    private void bindManualCoralClawToggle() {
        backup.rightBumper()
                .onTrue(manual.manualCoralClawToggle());
    }

    private void bindManualCoralIn() {
        backup.povDown()
                .whileTrue(manual.manualIntake(
                        Percent.of(0),
                        Percent.of(50)));
    }

    private void bindManualCoralOut() {
        backup.povUp()
                .whileTrue(manual.manualIntake(
                        Percent.of(0),
                        Percent.of(-25)));
    }

    private void bindManualAlgaeIn() {
        backup.povLeft()
                .whileTrue(manual.manualIntake(
                        Percent.of(-25),
                        Percent.of(-25)));
    }

    private void bindManualAlgaeOut() {
        backup.povRight()
                .whileTrue(manual.manualIntake(
                        Percent.of(25),
                        Percent.of(25)));
    }


    private Dimensionless getMaxVelocityPercent() {
        return maxVelocityPercent.mut_setBaseUnitMagnitude(getMaxVelocityPercentValue());
    }

    private double getMaxVelocityPercentValue() {
        double x = -MathUtil.applyDeadband(driver.getLeftX(), deadband);
        double y = -MathUtil.applyDeadband(driver.getLeftY(), deadband);
        double velocity = Math.hypot(x, y);
        return Math.min(velocity, 1);
    }

    private Dimensionless getMaxRotationalVelocityPercent() {
        return maxRotationalVelocityPercent.mut_setBaseUnitMagnitude(getMaxRotationalVelocityPercentValue());
    }

    private double getMaxRotationalVelocityPercentValue() {
        double leftTrigger = MathUtil.applyDeadband(driver.getLeftTriggerAxis(), deadband);
        double rightTrigger = MathUtil.applyDeadband(driver.getRightTriggerAxis(), deadband);
        return leftTrigger - rightTrigger;
    }

    private Angle getHeading() {
        return heading.mut_setBaseUnitMagnitude(getHeadingRadians());
    }

    private double getHeadingRadians() {
        double x = -MathUtil.applyDeadband(driver.getLeftX(), deadband);
        double y = -MathUtil.applyDeadband(driver.getLeftY(), deadband);
        return Math.atan2(y, x);
    }

    private Angle getRotation() {
        return rotation.mut_setBaseUnitMagnitude(getRotationRadians());
    }

    private double getRotationRadians() {
        double x = -MathUtil.applyDeadband(driver.getRightX(), deadband);
        double y = -MathUtil.applyDeadband(driver.getRightY(), deadband);
        return Math.atan2(y, x);
    }

    private Dimensionless getMaxWristVelocityPercent() {
        return maxManualWristVelocityPercent.mut_setBaseUnitMagnitude(getMaxWristVelocityPercentValue());
    }

    private double getMaxWristVelocityPercentValue() {
        double leftTrigger = MathUtil.applyDeadband(backup.getLeftTriggerAxis(), deadband);
        double rightTrigger = MathUtil.applyDeadband(backup.getRightTriggerAxis(), deadband);
        return leftTrigger - rightTrigger;
    }

    private Dimensionless getMaxElevatorVelocityPercent() {
        return maxManualElevatorVelocityPercent.mut_setBaseUnitMagnitude(getMaxElevatorVelocityPercentValue());
    }

    private double getMaxElevatorVelocityPercentValue() {
        return -MathUtil.applyDeadband(backup.getLeftY(), deadband);
    }

    private Dimensionless getMaxArmVelocityPercent() {
        return maxManualArmVelocityPercent.mut_setBaseUnitMagnitude(getMaxArmVelocityPercentValue());
    }

    private double getMaxArmVelocityPercentValue() {
        return -MathUtil.applyDeadband(backup.getRightY(), deadband);
    }
}
