package frc.robot.triggermaps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.game.CoralPickup;
import frc.robot.commands.game.CoralScore;
import frc.robot.commands.system.*;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.commands.system.Drive.ReefPosition.*;

public class TriggerMap {


    private final MutDimensionless maxVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxRotationalVelocityPercent = Value.mutable(0.0);

    private final MutAngle heading = Radians.mutable(0.0);
    private final MutAngle rotation = Radians.mutable(0.0);

    private final Drive driving;
    private final CoralScore scoring;

    public TriggerMap(Drive driving,
                      CoralScore scoring) {
        this.driving = driving;
        this.scoring = scoring;

        bindClockDrive();
        bindFieldCentricDrive();
        bindRobotCentricDrive();
        bindRotationSpeedDrive();
        bindZeroWheel();
        bindSeedFieldCentric();

        bindGoToNearestLeftReef();
        bindGoToNearestRightReef();
        bindGoToNearestAlgae();






        bindBargeAlign();
        bindBargeScore();
        bindProcessorAlign();
        bindProcessorScore();


        bindClimbScore();


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
                .and(driver.y())
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

    private void bindGoToNearestLeftReef() {
        driver.povLeft().and(driving.isNearestTag(17).or(driving.isNearestTag(8)))
                .whileTrue(driving.goToTag(17, LEFT));
        driver.povLeft().and(driving.isNearestTag(18).or(driving.isNearestTag(7)))
                .whileTrue(driving.goToTag(18, LEFT));
        driver.povLeft().and(driving.isNearestTag(19).or(driving.isNearestTag(6)))
                .whileTrue(driving.goToTag(19, LEFT));
        driver.povLeft().and(driving.isNearestTag(20).or(driving.isNearestTag(11)))
                .whileTrue(driving.goToTag(20, LEFT));
        driver.povLeft().and(driving.isNearestTag(21).or(driving.isNearestTag(10)))
                .whileTrue(driving.goToTag(21, LEFT));
        driver.povLeft().and(driving.isNearestTag(22).or(driving.isNearestTag(9)))
                .whileTrue(driving.goToTag(22, LEFT));
    }

    private void bindGoToNearestRightReef() {
        driver.povRight().and(driving.isNearestTag(17).or(driving.isNearestTag(8)))
                .whileTrue(driving.goToTag(17, RIGHT));
        driver.povRight().and(driving.isNearestTag(18).or(driving.isNearestTag(7)))
                .whileTrue(driving.goToTag(18, RIGHT));
        driver.povRight().and(driving.isNearestTag(19).or(driving.isNearestTag(6)))
                .whileTrue(driving.goToTag(19, RIGHT));
        driver.povRight().and(driving.isNearestTag(20).or(driving.isNearestTag(11)))
                .whileTrue(driving.goToTag(20, RIGHT));
        driver.povRight().and(driving.isNearestTag(21).or(driving.isNearestTag(10)))
                .whileTrue(driving.goToTag(21, RIGHT));
        driver.povRight().and(driving.isNearestTag(22).or(driving.isNearestTag(9)))
                .whileTrue(driving.goToTag(22, RIGHT));
    }

    private void bindGoToNearestAlgae() {
        driver.povUp().and(driving.isNearestTag(17).or(driving.isNearestTag(8)))
                .whileTrue(driving.goToTag(17, CENTER));
        driver.povUp().and(driving.isNearestTag(18).or(driving.isNearestTag(7)))
                .whileTrue(driving.goToTag(18, CENTER));
        driver.povUp().and(driving.isNearestTag(19).or(driving.isNearestTag(6)))
                .whileTrue(driving.goToTag(19, CENTER));
        driver.povUp().and(driving.isNearestTag(20).or(driving.isNearestTag(11)))
                .whileTrue(driving.goToTag(20, CENTER));
        driver.povUp().and(driving.isNearestTag(21).or(driving.isNearestTag(10)))
                .whileTrue(driving.goToTag(21, CENTER));
        driver.povUp().and(driving.isNearestTag(22).or(driving.isNearestTag(9)))
                .whileTrue(driving.goToTag(22, CENTER));
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




    private void bindClimbScore() {
        climb.axisMagnitudeGreaterThan(kRightY.value, deadband)
                .whileTrue(scoring.climb(this::getClimbDutyCycle));
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


}
