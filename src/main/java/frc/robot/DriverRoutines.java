package frc.robot;

import static frc.robot.RobotContainer.driverController;

public class DriverRoutines {

    private DriverRoutines() {
        // prevents instantiation
        // utility class
    }

    static void bind() {
        toggleDriveState();
        zeroWheels();
        brake();
        pointModules();
        seedFieldCentric();
    }

    private static void toggleDriveState() {
        // TODO: bind to driverController button either while or on True
    }

    private static void zeroWheels() {
        // TODO: bind to driverController button either while or on True
    }

    private static void brake() {
        // TODO: bind to driverController button either while or on True
    }

    private static void pointModules() {
        // TODO: bind to driverController button either while or on True
    }

    private static void seedFieldCentric() {
        // reset the field-centric heading on left bumper press
        // TODO: bind to driverController button either while or on True
    }




}
