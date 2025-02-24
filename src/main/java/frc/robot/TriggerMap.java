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
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class TriggerMap {
    private static final double deadband = 0.05;
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandJoystick climb = new CommandJoystick(2);
    private final CommandXboxController backup = new CommandXboxController(3);

    private final MutDimensionless maxVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxRotationalVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxManualWristVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxManualElevatorVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxManualArmVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless climbDutyCyclePercent = Value.mutable(0.0);
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
        bindFieldCentricDrive();
        bindRobotCentricDrive();
        bindRotationSpeedDrive();
        bindZeroWheel();
        bindSeedFieldCentric();

        bindGoToNearestReef();


        bindAlgaeFloorPickup();
        bindAlgaeL2Pickup();
        bindAlgaeL3Pickup();
        bindCoralFloorPickup();
        bindCoralStationPickup();

        bindBargeAlign();
        bindBargeScore();
        bindProcessorAlign();
        bindProcessorScore();
        bindL1Align();
        bindL2Align();
        bindL3Score();
        bindL4Score();
        bindL1Score();
        bindL234Score();

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
        new Trigger(() -> {
            double x = -MathUtil.applyDeadband(driver.getRightY(), deadband);
            double y = -MathUtil.applyDeadband(driver.getRightX(), deadband);
            return Math.hypot(x, y) != 0.0;
        }).whileTrue(driving.clockDrive(
                this::getMaxVelocityPercent,
                this::getHeading,
                this::getRotation));
    }

    private void bindFieldCentricDrive() {
        new Trigger(() -> getMaxVelocityPercentValue() != 0.0)
                .and(() -> {
                    double x = MathUtil.applyDeadband(driver.getRightY(), deadband);
                    double y = MathUtil.applyDeadband(driver.getRightX(), deadband);
                    return Math.hypot(x, y) == 0.0;
                })
                .whileTrue(driving.fieldCentricDrive(
                        this::getMaxVelocityPercent,
                        this::getHeading,
                        this::getMaxRotationalVelocityPercent));
    }

    private void bindRobotCentricDrive() {
        new Trigger(() -> getMaxVelocityPercentValue() != 0.0)
                .and(() -> {
                    double x = MathUtil.applyDeadband(driver.getRightY(), deadband);
                    double y = MathUtil.applyDeadband(driver.getRightX(), deadband);
                    return Math.hypot(x, y) == 0.0;
                })
                .and(driver.rightBumper())
                .whileTrue(driving.robotCentricDrive(
                        this::getMaxVelocityPercent,
                        this::getHeading,
                        this::getMaxRotationalVelocityPercent));
    }

    private void bindRotationSpeedDrive() {
        new Trigger(() -> getMaxVelocityPercentValue() == 0.0)
                .and(() -> {
                    double x = MathUtil.applyDeadband(driver.getRightY(), deadband);
                    double y = MathUtil.applyDeadband(driver.getRightX(), deadband);
                    return Math.hypot(x, y) == 0.0;
                })
                .whileTrue(driving.robotCentricDrive(
                        this::getMaxVelocityPercent,
                        this::getHeading,
                        this::getMaxRotationalVelocityPercent));
    }

    private void bindZeroWheel() {
        driver.back()
                .whileTrue(driving.zeroWheels());
    }

    private void bindSeedFieldCentric() {
        driver.start()
                .onTrue(driving.seedFieldCentric());
    }

    private void bindGoToNearestReef() {
        driver.povLeft().whileTrue(driving.goToNearestTag().cmd());
    }

    private void bindAlgaeFloorPickup() {
        operator.axisMagnitudeGreaterThan(kRightTrigger.value, deadband)
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
                .whileTrue(
                        sequence(pickup.coralFloor(), idle())
                                .withName("Pickup: Coral Floor"))
                .onFalse(pickup.coralStore());
    }

    private void bindCoralStationPickup() {
        operator.povUp()
                .whileTrue(
                        sequence(pickup.coralStation(), idle())
                                .withName("Pickup: Coral Station"))
                .onFalse(pickup.coralStore());
    }

    private void bindBargeAlign() {
        operator.povLeft()
                .whileTrue(scoring.bargeAlign());
    }

    private void bindBargeScore() {
        operator.axisMagnitudeGreaterThan(kLeftTrigger.value, deadband)
                .whileTrue(scoring.bargeScore())
                .onFalse(scoring.bargeScoreReset());
    }

    private void bindProcessorAlign() {
        operator.povDown().whileTrue(scoring.processorAlign())
                .onFalse(pickup.coralStore());
    }


    private void bindProcessorScore() {
        driver.rightBumper().whileTrue(scoring.processorScore());
    }

    private void bindL1Align() {
        operator.a()
                .whileTrue(scoring.l1Align())
                .onFalse(pickup.coralStore());
    }

    private void bindL2Align() {
        operator.x()
                .whileTrue(scoring.l2Align())
                .onFalse(pickup.coralStore());
    }

    private void bindL3Score() {
        operator.b()
                .whileTrue(scoring.l3Align())
                .onFalse(pickup.coralStore());
    }

    private void bindL4Score() {
        operator.y()
                .whileTrue(scoring.l4Align())
                .onFalse(pickup.coralStore());
    }

    private void bindL1Score() {
        driver.leftBumper().and(operator.a())
                .whileTrue(scoring.l1Score());
    }

    private void bindL234Score() {
        driver.leftBumper().and(operator.a().negate())
                .whileTrue(scoring.l2Score())
                .onFalse(scoring.L234ScoreReset());
    }


    private void bindClimbScore() {
        climb.axisMagnitudeGreaterThan(kY.value, deadband)
                .whileTrue(scoring.climb(this::getClimbDutyCycle));
    }

    private void bindManualWristToggle() {
        backup.y().whileTrue(manual.manualWristToggle());
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
                        Percent.of(50)).until(manual.hasCoral));
    }

    private void bindManualCoralOut() {
        backup.povUp()
                .whileTrue(manual.manualIntake(
                        Percent.of(0),
                        Percent.of(-25)));
    }

    private void bindManualAlgaeIn() {
        backup.povLeft()
                .whileTrue(
                        sequence(
                                race(manual.manualIntake(Percent.of(-100), Percent.of(-100)), waitSeconds(0.5)),
                                manual.manualIntake(Percent.of(-100), Percent.of(-100)).until(manual.hasAlgae)));
    }

    private void bindManualAlgaeOut() {
        backup.povRight()
                .whileTrue(manual.manualIntake(
                        Percent.of(100),
                        Percent.of(100)));
    }

    private Dimensionless getMaxVelocityPercent() {
        return maxVelocityPercent.mut_setBaseUnitMagnitude(getMaxVelocityPercentSqValue());
    }

    private double getMaxVelocityPercentValue() {
        double x = -MathUtil.applyDeadband(driver.getLeftY(), deadband);
        double y = -MathUtil.applyDeadband(driver.getLeftX(), deadband);
        double velocity = Math.hypot(x, y);
        return Math.min(velocity, 1);
    }

    private double getMaxVelocityPercentSqValue() {
        double x = -MathUtil.applyDeadband(driver.getLeftY(), deadband);
        double y = -MathUtil.applyDeadband(driver.getLeftX(), deadband);
        double velocity = Math.hypot(x, y);
        return Math.pow(Math.min(velocity, 1), 2);
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
        double x = -MathUtil.applyDeadband(driver.getLeftY(), deadband);
        double y = -MathUtil.applyDeadband(driver.getLeftX(), deadband);
        return Math.atan2(y, x);
    }

    private Angle getRotation() {
        return rotation.mut_setBaseUnitMagnitude(getRotationRadians());
    }

    private double getRotationRadians() {
        double x = -MathUtil.applyDeadband(driver.getRightY(), deadband);
        double y = -MathUtil.applyDeadband(driver.getRightX(), deadband);
        return Math.atan2(y, x);
    }

    private Dimensionless getMaxWristVelocityPercent() {
        return maxManualWristVelocityPercent.mut_setBaseUnitMagnitude(getMaxWristVelocityPercentValue());
    }

    private double getMaxWristVelocityPercentValue() {
        double leftTrigger = MathUtil.applyDeadband(backup.getLeftTriggerAxis(), deadband);
        double rightTrigger = MathUtil.applyDeadband(backup.getRightTriggerAxis(), deadband);
        return 0.2 * (leftTrigger - rightTrigger);
    }

    private Dimensionless getMaxElevatorVelocityPercent() {
        return maxManualElevatorVelocityPercent.mut_setBaseUnitMagnitude(getMaxElevatorVelocityPercentValue());
    }

    private double getMaxElevatorVelocityPercentValue() {
        return -0.2 * MathUtil.applyDeadband(backup.getLeftY(), deadband);
    }

    private Dimensionless getMaxArmVelocityPercent() {
        return maxManualArmVelocityPercent.mut_setBaseUnitMagnitude(getMaxArmVelocityPercentValue());
    }

    private double getMaxArmVelocityPercentValue() {
        return -0.2 * MathUtil.applyDeadband(backup.getRightY(), deadband);
    }

    private Dimensionless getClimbDutyCycle() {
        return climbDutyCyclePercent.mut_setBaseUnitMagnitude(getClimbDutyCycleValue());
    }

    private double getClimbDutyCycleValue() {
        return -MathUtil.applyDeadband(climb.getY(), deadband);
    }
}
