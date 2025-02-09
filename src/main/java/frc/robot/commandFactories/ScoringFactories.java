package frc.robot.commandFactories;

import digilib.controllers.DriverCommandXBoxController;
import digilib.controllers.OperatorFlightStickController;
import digilib.controllers.OperatorXboxController;

public class ScoringFactories {

    private DriverCommandXBoxController driverController;
    private OperatorXboxController operatorXboxController;
    private OperatorFlightStickController operatorFlightController;

    public ScoringFactories(
            DriverCommandXBoxController driverController,
            OperatorXboxController operatorXboxController,
            OperatorFlightStickController operatorFlightStickController
    ) {
        this.driverController = driverController;
        this.operatorXboxController = operatorXboxController;
        this.operatorFlightController = operatorFlightStickController;
    }

}
