package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorFlightStickController {

    private final double deadband;
    private final CommandXboxController controller;

    public OperatorFlightStickController(double deadband) {
        this.deadband = deadband;
        controller = new CommandXboxController(2);
    }

}
