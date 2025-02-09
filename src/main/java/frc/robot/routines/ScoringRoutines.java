package frc.robot.routines;

import digilib.controllers.OperatorXboxController;
import frc.robot.Constants;
import frc.robot.commandFactories.PickupFactories;
import frc.robot.commandFactories.SubsystemCommandFactories;

public class ScoringRoutines {
    private final OperatorXboxController operatorController = new OperatorXboxController(Constants.controllerDeadband);
    private final SubsystemCommandFactories subsystemCommandFactories;
    private final PickupFactories pickupFactories;

    public ScoringRoutines(
            SubsystemCommandFactories subsystemCommandFactories,
            PickupFactories pickupFactories) {
        this.pickupFactories = pickupFactories;
        this.subsystemCommandFactories = subsystemCommandFactories;
    }
}
