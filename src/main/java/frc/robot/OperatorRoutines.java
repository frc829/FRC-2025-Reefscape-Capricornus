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
        elevator();
        wrist();
    }

    private static void elevator(){
        operatorController.b().whileTrue(CommandFactory.ElevatorCommands.testCommand());
    }
    private static void arm(){
        operatorController.a().whileTrue(CommandFactory.ArmCommands.testCommand());
    }



    private static void wrist(){

    }
}
