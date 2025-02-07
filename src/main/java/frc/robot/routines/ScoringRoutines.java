package frc.robot.routines;

import digilib.controllers.OperatorXboxController;
import frc.robot.Constants;
import frc.robot.commandFactories.AlgaePickupFactories;
import frc.robot.commandFactories.SubsystemCommandFactories;

public class ScoringRoutines {
    private final OperatorXboxController operatorController = new OperatorXboxController(Constants.controllerDeadband);
    private final SubsystemCommandFactories subsystemCommandFactories;
    private final AlgaePickupFactories algaePickupFactories;

    public ScoringRoutines(
            SubsystemCommandFactories subsystemCommandFactories,
            AlgaePickupFactories algaePickupFactories) {
        this.algaePickupFactories = algaePickupFactories;
        this.subsystemCommandFactories = subsystemCommandFactories;
    }
}
