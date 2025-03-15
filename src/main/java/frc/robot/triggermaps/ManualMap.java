package frc.robot.triggermaps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Manual;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ManualMap {
    private static final double maxManualArmSetpointScalar = 0.2;
    private static final double maxManualElevatorSetpointScalar = 0.2;
    private static final double maxManualWristSetpointScalar = 1.0;

    private final CommandXboxController operator;
    private final CommandXboxController controller;
    private final double deadband;
    private final Manual manual;

    public ManualMap(CommandXboxController controller,
                     CommandXboxController operator,
                     double deadband,
                     Manual manual) {
        this.controller = controller;
        this.operator = operator;
        this.deadband = deadband;
        this.manual = manual;
        bindManualArm();
        bindArm60Test();
        bindArm0Test();

        bindManualAlgaeClawToggle();
        bindManualCoralClawToggle();

        bindManualElevator();
        bindManualElevatorTest();
        bindManualElevatorDownTest();

        bindManualWristToggle();
        bindManualWrist();

        bindManualCoralIn();
        bindManualCoralOut();

        bindManualAlgaeIn();
        bindManualAlgaeOut();

        bindSteer90Test();
        bindSteer0Test();

        bindManualElevatorDangerous();

    }

    private void bindManualArm() {
        new Trigger(() -> getArmVelocitySetpointScalar() != 0.0)
                .whileTrue(manual.manualArm(this::getArmVelocitySetpointScalar));

        new Trigger(() -> getArmVelocitySetpointScalarManual() != 0.0)
                .whileTrue(manual.manualArm(this::getArmVelocitySetpointScalarManual));
    }

    private double getArmVelocitySetpointScalar() {
        return maxManualArmSetpointScalar * MathUtil.applyDeadband(-operator.getRightY(), deadband);
    }

    private double getArmVelocitySetpointScalarManual() {
        return maxManualArmSetpointScalar * MathUtil.applyDeadband(-controller.getRightY(), deadband);
    }

    private void bindArm60Test() {
        controller.b().whileTrue(manual.manualArm60Test());
    }

    private void bindArm0Test() {
        controller.back().whileTrue(manual.manualArm0Test());
    }


    private void bindManualAlgaeClawToggle() {
        controller.leftBumper()
                .onTrue(manual.manualAlgaeClawToggle());
    }

    private void bindManualCoralClawToggle() {
        controller.rightBumper()
                .onTrue(manual.manualCoralClawToggle());
    }

    private void bindManualElevator() {
        new Trigger(() -> getMaxElevatorVelocityPercentValue() != 0.0)
                .whileTrue(manual.manualElevator(this::getMaxElevatorVelocityPercentValue));

        new Trigger(() -> getMaxElevatorVelocityPercentValueManual() != 0.0)
                .whileTrue(manual.manualElevator(this::getMaxElevatorVelocityPercentValueManual));
    }

    private void bindManualElevatorDangerous(){
        new Trigger(() -> getMaxElevatorVelocityPercentValueManual() != 0.0).and(controller.start())
                .whileTrue(manual.manualElevatorDangerous(() -> 0.1 * 12.0 * getMaxElevatorVelocityPercentValueManual()));
    }

    private double getMaxElevatorVelocityPercentValue() {
        return maxManualElevatorSetpointScalar * MathUtil.applyDeadband(-operator.getLeftY(), deadband);
    }

    private double getMaxElevatorVelocityPercentValueManual() {
        return maxManualElevatorSetpointScalar * MathUtil.applyDeadband(-controller.getLeftY(), deadband);
    }

    private void bindManualElevatorTest() {
        controller.a().whileTrue(manual.manualElevatorTest());
    }

    private void bindManualElevatorDownTest() {
        controller.x().whileTrue(manual.manualElevatorTestDown());
    }

    private void bindManualAlgaeIn() {
        controller.povLeft()
                .whileTrue(
                        sequence(
                                race(
                                        parallel(
                                                manual.manualAlgaeIntake(() -> -1),
                                                manual.manualCoralIntake(() -> -1)),
                                        waitSeconds(0.5)),
                                parallel(
                                        manual.manualAlgaeIntake(() -> -1),
                                        manual.manualCoralIntake(() -> -1))
                                        .until(manual.hasAlgae)));
    }

    private void bindManualAlgaeOut() {
        controller.povRight()
                .whileTrue(
                        parallel(
                                manual.manualAlgaeIntake(() -> 1),
                                manual.manualCoralIntake(() -> 1)));
    }

    private void bindManualCoralIn() {
        controller.povDown()
                .whileTrue(
                        manual.manualCoralIntake(() -> 1)
                                .until(manual.hasCoral));
    }

    private void bindManualCoralOut() {
        controller.povUp()
                .whileTrue(
                        manual.manualCoralIntake(() -> 1));
    }

    private void bindManualWristToggle() {
        controller.y().whileTrue(manual.manualWristToggle());
    }

    private void bindManualWrist() {
        new Trigger(() -> getWristVelocitySetpointScalar() != 0.0)
                .whileTrue(manual.manualWrist(this::getWristVelocitySetpointScalar));
    }

    private void bindSteer90Test(){
        controller.a().and(controller.b()).whileTrue(manual.manualSteer90Test());
    }

    private void bindSteer0Test(){
        controller.x().and(controller.y()).whileTrue(manual.manualSteer0Test());
    }

    private double getWristVelocitySetpointScalar() {
        double leftTrigger = MathUtil.applyDeadband(controller.getLeftTriggerAxis(), deadband);
        double rightTrigger = MathUtil.applyDeadband(controller.getRightTriggerAxis(), deadband);
        return maxManualWristSetpointScalar * (leftTrigger - rightTrigger);
    }
}
