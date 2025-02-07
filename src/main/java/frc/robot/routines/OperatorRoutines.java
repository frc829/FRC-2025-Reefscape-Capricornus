package frc.robot.routines;

import digilib.controllers.OperatorCommandXBoxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commandFactories.AlgaePickupFactories;
import frc.robot.commandFactories.SubsystemCommandFactories;
import frc.robot.subsystems.algaeClaw.CommandAlgaeClawFactory;
import frc.robot.subsystems.arm.CommandArmFactory;
import frc.robot.subsystems.coralClaw.CommandCoralClawFactory;
import frc.robot.subsystems.dualIntake.CommandDualIntakeFactory;
import frc.robot.subsystems.elevator.CommandElevatorFactory;
import frc.robot.subsystems.hook.CommandHookFactory;
import frc.robot.subsystems.winch.CommandWinchFactory;
import frc.robot.subsystems.wrist.CommandWristFactory;

public class OperatorRoutines {
    private final OperatorCommandXBoxController operatorController = new OperatorCommandXBoxController(Constants.controllerDeadband);
    private final SubsystemCommandFactories subsystemCommandFactories;
    private final AlgaePickupFactories algaePickupFactories;

    public OperatorRoutines(
            SubsystemCommandFactories subsystemCommandFactories,
            AlgaePickupFactories algaePickupFactories) {
        this.algaePickupFactories = algaePickupFactories;
        this.subsystemCommandFactories = subsystemCommandFactories;
    }
}
