package frc.robot;

import static frc.robot.RobotContainer.commandArm;
import static frc.robot.RobotContainer.commandSwerveDrive;

public class DefaultRoutines {

    public DefaultRoutines(){
        // prevents instantiation
        // utility class
    }

    static void bind(){
        drive();
        arm();
    }

    private static void drive(){
        commandSwerveDrive.setDefaultCommand(CommandFactory.DriveCommands.defaultDrive());
    }

    private static void arm() {
        commandArm.setDefaultCommand(CommandFactory.ArmCommands.defaultArm());
    }
}
