package frc.robot.routines;

import frc.robot.controllers.ManualController;
import frc.robot.commandFactories.SubsystemCommandFactories;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

public class ManualRoutines {
    private final ManualController controller;
    private final SubsystemCommandFactories factories;

    public ManualRoutines(SubsystemCommandFactories factories,
                          ManualController controller) {
        this.factories = factories;
        this.controller = controller;
        arm();
        elevator();
        wrist();
        wristToggle();
        toggleAlgaeClaw();
        toggleCoralClaw();
        coralIn();
        coralOut();
        algaeIn();
        algaeOut();
        armTo30Deg();
        elevatorTo20cm();
    }

    private void wristToggle() {
        controller.wristToggle().onTrue(factories.wrist.toggle());
    }

    private void arm() {
        controller.arm().whileTrue(factories.arm.moveAtVelocity(controller::getArmVelocity)
                .withName(String.format("%s: VELOCITY", factories.arm.getName())));
    }

    private void elevator() {
        controller.elevator().whileTrue(factories.elevator.moveAtVelocity(controller::getElevatorVelocity));
    }

    private void wrist() {
        controller.wrist().whileTrue(factories.wrist.moveAtVelocity(controller::getWristVelocity));
    }

    private void toggleAlgaeClaw() {
        controller.algaeClawToggle().whileTrue(factories.algae.toggle());
    }

    private void toggleCoralClaw() {
        controller.coralClawToggle().whileTrue(factories.coral.toggle());
    }

    private void coralIn() {
        controller.coralIn().whileTrue(factories.dualIntake.moveAtVelocity(
                        () -> 0.0,
                        () -> 0.50)
                .until(factories.dualIntake.hasCoral)
                .withName("Manual: CORAL IN"));
    }

    public void coralOut() {
        controller.coralOut().whileTrue(factories.dualIntake.moveAtVelocity(
                        () -> 0.0,
                        () -> -0.25)
                .withName("Manual: CORAL OUT"));
    }

    private void algaeIn() {
        controller.algaeIn().whileTrue(factories.dualIntake.moveAtVelocity(
                        () -> -0.25,
                        () -> -0.25)
                .until(factories.dualIntake.hasAlgae)
                .withName("Manual: ALGAE IN"));
    }

    private void algaeOut() {
        controller.algaeOut().whileTrue(factories.dualIntake.moveAtVelocity(
                        () -> 0.25,
                        () -> 0.25)
                .withName("Manual: ALGAE OUT"));
    }

    private void elevatorTo20cm() {
        controller.testElevatorPos()
                .whileTrue(
                        factories.elevator.goToPosition(Centimeters.of(20.0), Centimeters.of(2.0))
                                .withName("Elevator to 20 cm"));
    }

    private void armTo30Deg() {
        controller.testArmPos()
                .whileTrue((factories.arm.goToAngle(Degrees.of(30), Degrees.of(2.0)))
                        .withName("Arm to 30 deg"));
    }

}
