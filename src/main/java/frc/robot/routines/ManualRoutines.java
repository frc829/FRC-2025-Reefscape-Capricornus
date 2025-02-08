package frc.robot.routines;

import digilib.controllers.ManualController;
import frc.robot.Constants;
import frc.robot.commandFactories.SubsystemCommandFactories;

public class ManualRoutines {
    private final ManualController controller = new ManualController(Constants.controllerDeadband);
    private final SubsystemCommandFactories factories;

    public ManualRoutines(SubsystemCommandFactories factories) {
        this.factories = factories;
        arm();
        elevator();
        wrist();
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

}
