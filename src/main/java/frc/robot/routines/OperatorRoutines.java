package frc.robot.routines;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ComboCommandFactory;
import frc.robot.subsystems.CommandArm;
import frc.robot.subsystems.CommandElevator;

public class OperatorRoutines {

    private final CommandXboxController operatorController;
    private final ComboCommandFactory commandFactory;
    private final CommandArm commandArm;
    private final CommandElevator commandElevator;

    public OperatorRoutines(CommandXboxController operatorController,
                            ComboCommandFactory commandFactory,
                            CommandArm commandArm,
                            CommandElevator commandElevator) {
        this.operatorController = operatorController;
        this.commandFactory = commandFactory;
        this.commandArm = commandArm;
        this.commandElevator = commandElevator;
        elevator();
        arm();
    }


    private void elevator(){
        operatorController.b().whileTrue(commandFactory.testCommand());
    }
    private void arm(){
        operatorController.a().whileTrue(commandFactory.testCommand());
    }
}
