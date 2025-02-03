package frc.robot;

import static frc.robot.RobotContainer.commandSwerveDrive;
import static frc.robot.RobotContainer.driverController;

public class DriverRoutines {

    private DriverRoutines() {
        // prevents instantiation
        // utility class
    }

    static void bind() {
        zeroWheels();
        brake();
        pointModules();
        seedFieldCentric();
        goToReef0();
        toggleClock();
    }

    private static void zeroWheels() {
        driverController.back().whileTrue(CommandFactory.DriveCommands.zeroWheels());
    }

    private static void brake() {
        driverController.x().whileTrue(CommandFactory.DriveCommands.brake());
    }

    private static void pointModules() {
        driverController.y().whileTrue(CommandFactory.DriveCommands.pointModuleWheels());
    }

    private static void seedFieldCentric() {
        driverController.start().whileTrue(CommandFactory.DriveCommands.seedFieldCentric());
    }

    private static void goToReef0(){
        driverController.a().whileTrue(CommandFactory.DriveCommands.goToReef0());
    }


    private static void toggleClock() {
        driverController.x().onTrue(commandSwerveDrive.toggleClock());
    }






}
