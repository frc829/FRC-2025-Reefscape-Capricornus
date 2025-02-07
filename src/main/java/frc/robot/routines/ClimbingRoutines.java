package frc.robot.routines;

import digilib.controllers.OperatorFlightStickController;
import frc.robot.commandFactories.SubsystemCommandFactories;

public class ClimbingRoutines {

    private final OperatorFlightStickController flightStick;
    private final SubsystemCommandFactories subsystemCommandFactories;

    public ClimbingRoutines(
            OperatorFlightStickController flightStick,
            SubsystemCommandFactories subsystemCommandFactories) {
        this.flightStick = flightStick;
        this.subsystemCommandFactories = subsystemCommandFactories;
    }

}
