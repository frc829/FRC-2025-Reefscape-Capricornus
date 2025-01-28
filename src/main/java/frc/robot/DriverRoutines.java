package frc.robot;

import static frc.robot.RobotContainer.driverController;

public class DriverRoutines {

    private DriverRoutines() {
        // prevents instantiation
        // utility class
    }

    static void bind() {
        fieldCentric();
        robotCentric();
        zeroWheels();
        brake();
        pointModules();
        seedFieldCentric();
    }

    private static void fieldCentric() {
        driverController.rightBumper().whileTrue(CommandFactory.DriveCommands.fieldCentricDrive());
    }

    private static void robotCentric() {
        driverController.pov(0).whileTrue(CommandFactory.DriveCommands.robotCentricForward());
        driverController.pov(180).whileTrue(CommandFactory.DriveCommands.robotCentricReverse());
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
