package frc.robot.routines;

import digilib.controllers.DriverController;
import frc.robot.Constants;
import frc.robot.commandFactories.SubsystemCommandFactories;

public class DriverRoutines {

    private final SubsystemCommandFactories subsystemCommandFactories;
    private final DriverController driverController;


    public DriverRoutines(SubsystemCommandFactories subsystemCommandFactories,
                          DriverController driverController) {
        this.subsystemCommandFactories = subsystemCommandFactories;
        this.driverController = driverController;

        clockDrive();
    }

    private void clockDrive() {
        driverController.getClockDriveTrigger().whileTrue(
                subsystemCommandFactories.swerve.clockDrive(
                        driverController::getVelocity,
                        driverController::getHeading,
                        driverController::getRotation
                )
        );
    }


}
