package frc.robot.routines;

import frc.robot.controllers.OperatorXboxController;
import frc.robot.commandFactories.PickupFactories;
import frc.robot.commandFactories.ResetFactories;

public class PickupRoutines {
    private final OperatorXboxController controller;
    private final PickupFactories factories;
    private final ResetFactories resetFactories;

    public PickupRoutines(
            OperatorXboxController controller,
            PickupFactories factories,
            ResetFactories resetFactories) {
        this.controller = controller;
        this.factories = factories;
        this.resetFactories = resetFactories;

        // controller.algaeFloor()
        //         .whileTrue(factories.algaeFloor())
        //         .onFalse(resetFactories.algae());
        // controller.algaeL2()
        //         .whileTrue(factories.algaeL2())
        //         .onFalse(resetFactories.algae());
        // controller.algaeL3()
        //         .whileTrue(factories.algaeL3())
        //         .onFalse(resetFactories.algae());
        // controller.coralFloor()
        //         .whileTrue(factories.coralFloor())
        //         .onFalse(resetFactories.coral());
        controller.coralStationFront()
                .whileTrue(factories.coralStation());
        // controller.coralStationBack()
        //         .whileTrue(factories.coralStationBack())
        //         .onFalse(resetFactories.coral());
    }
}
