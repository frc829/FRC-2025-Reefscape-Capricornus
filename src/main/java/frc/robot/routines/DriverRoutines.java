package frc.robot.routines;

import digilib.controllers.DriverCommandXBoxController;
import frc.robot.Constants;
import frc.robot.commandFactories.SubsystemCommandFactories;

public class DriverRoutines {

    private final SubsystemCommandFactories subsystemCommandFactories;
    private final DriverCommandXBoxController driverController = new DriverCommandXBoxController(Constants.controllerDeadband);


    public DriverRoutines(SubsystemCommandFactories subsystemCommandFactories) {
        this.subsystemCommandFactories = subsystemCommandFactories;
    }
}
