package frc.robot.routines;

import digilib.controllers.DriverCommandXBoxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commandFactories.SubsystemCommandFactories;
import frc.robot.subsystems.algaeClaw.CommandAlgaeClawFactory;
import frc.robot.subsystems.arm.CommandArmFactory;
import frc.robot.subsystems.coralClaw.CommandCoralClawFactory;
import frc.robot.subsystems.dualIntake.CommandDualIntakeFactory;
import frc.robot.subsystems.elevator.CommandElevatorFactory;
import frc.robot.subsystems.hook.CommandHookFactory;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveFactory;
import frc.robot.subsystems.winch.CommandWinchFactory;
import frc.robot.subsystems.wrist.CommandWristFactory;

public class DriverRoutines {

    private final SubsystemCommandFactories subsystemCommandFactories;
    private final DriverCommandXBoxController driverController = new DriverCommandXBoxController(Constants.controllerDeadband);


    public DriverRoutines(SubsystemCommandFactories subsystemCommandFactories) {
        this.subsystemCommandFactories = subsystemCommandFactories;
    }
}
