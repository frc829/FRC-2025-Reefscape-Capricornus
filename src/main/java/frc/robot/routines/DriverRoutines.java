package frc.robot.routines;

import frc.robot.controllers.DriverController;
import frc.robot.commandFactories.SubsystemCommandFactories;

public class DriverRoutines {

    private final SubsystemCommandFactories factories;
    private final DriverController driverController;


    public DriverRoutines(SubsystemCommandFactories factories,
                          DriverController driverController) {
        this.factories = factories;
        this.driverController = driverController;

        clockDrive();
    }

    private void clockDrive() {
        driverController.getClockDriveTrigger().whileTrue(
                factories.swerve.clockDrive(
                        driverController::getVelocity,
                        driverController::getHeading,
                        driverController::getRotation
                )
        );
        driverController.getRobotCentricTrigger().whileTrue(
                factories.swerve.robotCentricDrive(
                        driverController::getVelocity,
                        driverController::getRotationalVelocity,
                        driverController::getHeading
                )
        );
    }


}
