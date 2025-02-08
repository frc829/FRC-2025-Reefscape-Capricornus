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
    }

    private void arm() {
        controller.arm().whileTrue(factories.arm.moveAtVelocity(controller::getArmVelocity));
    }
}
