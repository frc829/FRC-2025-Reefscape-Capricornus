package frc.robot.routines;

import digilib.controllers.DriverController;
import frc.robot.Constants;
import frc.robot.commandFactories.SubsystemCommandFactories;

public class DriverRoutines {

    private final SubsystemCommandFactories subsystemCommandFactories;
    private final DriverController driverController = new DriverController(Constants.controllerDeadband);


    public DriverRoutines(SubsystemCommandFactories subsystemCommandFactories) {
        this.subsystemCommandFactories = subsystemCommandFactories;
    }
}
