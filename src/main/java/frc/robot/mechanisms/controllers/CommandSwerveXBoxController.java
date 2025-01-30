package frc.robot.mechanisms.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandSwerveXBoxController extends CommandXboxController {

    private final double deadband;

    public CommandSwerveXBoxController(int port, double deadband) {
        super(port);
        this.deadband = deadband;
    }

    public double forward() {
        return 0.0; // TODO: remove this when done
    }

    public double strafe() {
        return 0.0; // TODO: remove this when done.
    }

    public double turn() {
        return 0.0; // TODO: remove this when done.
    }

    public double clockAngle() {
        return 0.0; // TODO: remove this when done.
    }

}
