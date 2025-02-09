package digilib.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorXboxController {

    private final double deadband;
    private final CommandXboxController controller;

    public OperatorXboxController(double deadband) {
        this.deadband = deadband;
        controller = new CommandXboxController(1);
    }

    public Trigger algaeFloor() {
        return controller.axisMagnitudeGreaterThan(XboxController.Axis.kRightTrigger.value, deadband);
    }

    public Trigger algaeL2() {
        return controller.rightStick();
    }

    public Trigger algaeL3() {
        return controller.leftStick();
    }

    public Trigger coralFloor() {
        return controller.rightBumper();
    }

    public Trigger coralStationFront() {
        return controller.povUp();
    }

    public Trigger coralStationBack() {
        return controller.leftBumper().and(controller.povUp());
    }

}
