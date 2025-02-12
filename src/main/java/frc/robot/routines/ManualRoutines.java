package frc.robot.routines;

import digilib.controllers.ManualController;
import frc.robot.commandFactories.SubsystemCommandFactories;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Value;

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
        testWrist0();
        testWrist90();
        toggleAlgaeClaw();
        toggleCoralClaw();
        coralIn();
        coralOut();
        algaeIn();
        algaeOut();
    }

    private void testWrist90() {
        controller.testWristPose90().whileTrue(factories.wrist.goToAngle(Degrees.of(90.0)));
    }

    private void testWrist0() {
        controller.testWristPose0().whileTrue(factories.wrist.goToAngle(Degrees.of(0.0)));
    }

    private void arm() {
        controller.arm().whileTrue(factories.arm.moveAtVelocity(controller::getArmVelocity));
    }

    private void elevator() {
        controller.elevator().whileTrue(factories.elevator.moveAtVelocity(controller::getElevatorVelocity));
    }

    private void wrist() {
        controller.wrist().whileTrue(factories.wrist.moveAtVelocity(controller::getWristVelocity));
    }

    private void toggleAlgaeClaw() {
        controller.algaeClawToggle().whileTrue(factories.algae.toggleClaw());
    }

    private void toggleCoralClaw() {
        controller.coralClawToggle().whileTrue(factories.coral.toggleClaw());
    }

    private void coralIn() {
        controller.coralIn().whileTrue(factories.intake.moveAtVelocity(
                () -> Value.of(0.0),
                () -> Value.of(0.25)
        ).until(factories.intake.hasCoral));
    }

    public void coralOut() {
        controller.coralOut().whileTrue(factories.intake.moveAtVelocity(
                () -> Value.of(0.0),
                () -> Value.of(-0.25)
        ));
    }

    private void algaeIn() {
        controller.algaeIn().whileTrue(factories.intake.moveAtVelocity(
                () -> Value.of(-0.25),
                () -> Value.of(-0.25)
        ));
    }

    private void algaeOut() {
        controller.algaeOut().whileTrue(factories.intake.moveAtVelocity(
                () -> Value.of(0.25),
                () -> Value.of(0.25)
        ));
    }

}
