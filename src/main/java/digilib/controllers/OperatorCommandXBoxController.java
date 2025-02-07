package digilib.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorCommandXBoxController {

    private final double deadband;
    private final CommandXboxController controller;

    public OperatorCommandXBoxController(double deadband) {
        this.deadband = deadband;
        controller = new CommandXboxController(1);
    }

}
