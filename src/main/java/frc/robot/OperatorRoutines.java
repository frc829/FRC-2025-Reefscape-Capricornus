package frc.robot;

import static frc.robot.RobotContainer.operatorController;

public class OperatorRoutines {

    private OperatorRoutines(){
        // prevents instantiation
        // utility class
    }


    static void bind(){
        elevator();
        arm();
        wrist();
    }

    private static void elevator(){

    }

    private static void arm(){
        operatorController.a().whileTrue(CommandFactory.ArmCommands.position5Degrees());
    }

    private static void wrist(){

    }
}
