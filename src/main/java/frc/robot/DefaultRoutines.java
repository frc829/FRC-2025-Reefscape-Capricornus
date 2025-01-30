package frc.robot;

import static frc.robot.RobotContainer.commandSwerveDrive;

public class DefaultRoutines {

    public DefaultRoutines(){
        // prevents instantiation
        // utility class
    }

    static void bind(){
        drive();
    }

    private static void drive(){
        commandSwerveDrive.setDefaultCommand(CommandFactory.DriveCommands.defaultDrive());
    }
}
