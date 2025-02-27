package frc.robot.triggermaps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.game.Manual;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ManualMap {
    private final CommandXboxController controller;
    private final double deadband;
    private final Manual manual;

    private final MutDimensionless maxManualWristVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxManualElevatorVelocityPercent = Value.mutable(0.0);
    private final MutDimensionless maxManualArmVelocityPercent = Value.mutable(0.0);

    public ManualMap(CommandXboxController controller,
                     double deadband,
                     Manual manual) {
        this.controller = controller;
        this.deadband = deadband;
        this.manual = manual;
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

        bindManualElevatorTest();
        bindArmTest();
    }

    private void bindManualWristToggle() {
        controller.y().whileTrue(manual.manualWristToggle());
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
        controller.leftBumper()
                .onTrue(manual.manualAlgaeClawToggle());
    }

    private void bindManualCoralClawToggle() {
        controller.rightBumper()
                .onTrue(manual.manualCoralClawToggle());
    }

    private void bindManualCoralIn() {
        controller.povDown()
                .whileTrue(manual.manualIntake(
                        Percent.of(0),
                        Percent.of(50)).until(manual.hasCoral));
    }

    private void bindManualCoralOut() {
        controller.povUp()
                .whileTrue(manual.manualIntake(
                        Percent.of(0),
                        Percent.of(-25)));
    }

    private void bindManualAlgaeIn() {
        controller.povLeft()
                .whileTrue(
                        sequence(
                                race(manual.manualIntake(Percent.of(-100), Percent.of(-100)), waitSeconds(0.5)),
                                manual.manualIntake(Percent.of(-100), Percent.of(-100)).until(manual.hasAlgae)));
    }

    private void bindManualAlgaeOut() {
        controller.povRight()
                .whileTrue(manual.manualIntake(
                        Percent.of(100),
                        Percent.of(100)));
    }

    private Dimensionless getMaxWristVelocityPercent() {
        return maxManualWristVelocityPercent.mut_setBaseUnitMagnitude(getMaxWristVelocityPercentValue());
    }

    private double getMaxWristVelocityPercentValue() {
        double leftTrigger = MathUtil.applyDeadband(controller.getLeftTriggerAxis(), deadband);
        double rightTrigger = MathUtil.applyDeadband(controller.getRightTriggerAxis(), deadband);
        return 1 * (leftTrigger - rightTrigger);
    }

    private Dimensionless getMaxElevatorVelocityPercent() {
        return maxManualElevatorVelocityPercent.mut_setBaseUnitMagnitude(getMaxElevatorVelocityPercentValue());
    }

    private double getMaxElevatorVelocityPercentValue() {
        return -0.2 * MathUtil.applyDeadband(controller.getLeftY(), deadband);
    }

    private Dimensionless getMaxArmVelocityPercent() {
        return maxManualArmVelocityPercent.mut_setBaseUnitMagnitude(getMaxArmVelocityPercentValue());
    }

    private double getMaxArmVelocityPercentValue() {
        return -0.2 * MathUtil.applyDeadband(controller.getRightY(), deadband);
    }

    private void bindManualElevatorTest(){
        controller.a().whileTrue(manual.manualElevatorTest());
    }

    private void bindArmTest(){
        controller.b().whileTrue(manual.manualArmTest());
    }
}
