package frc.robot.commandFactories;

import digilib.controllers.DriverController;
import digilib.controllers.OperatorFlightStickController;
import digilib.controllers.OperatorXboxController;

public class ScoringFactories {

    private DriverController driverController;
    private OperatorXboxController operatorXboxController;
    private OperatorFlightStickController operatorFlightController;

    public ScoringFactories(
            DriverController driverController,
            OperatorXboxController operatorXboxController,
            OperatorFlightStickController operatorFlightStickController
    ) {
        this.driverController = driverController;
        this.operatorXboxController = operatorXboxController;
        this.operatorFlightController = operatorFlightStickController;
    }

}
