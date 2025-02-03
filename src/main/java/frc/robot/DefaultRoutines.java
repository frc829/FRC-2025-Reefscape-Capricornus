package frc.robot;

import static frc.robot.RobotContainer.*;

public class DefaultRoutines {

    public DefaultRoutines(){
        // prevents instantiation
        // utility class
    }

    static void bind(){
        drive();
        arm();
        elevator();
    }

    private static void drive(){
        commandSwerveDrive.setDefaultCommand(CommandFactory.DriveCommands.defaultDrive());
    }

    private static void arm() {
        commandArm.setDefaultCommand(CommandFactory.ArmCommands.defaultArm());
    }

    private static void elevator() {
        commandElevator.setDefaultCommand(CommandFactory.ElevatorCommands.defaultElevator());
    }
}
