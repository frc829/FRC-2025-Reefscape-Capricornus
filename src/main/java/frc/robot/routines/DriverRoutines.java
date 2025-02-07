package frc.robot.routines;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SubsystemCommandFactories;
import frc.robot.subsystems.algaeClaw.CommandAlgaeClawFactory;
import frc.robot.subsystems.arm.CommandArmFactory;
import frc.robot.subsystems.coralClaw.CommandCoralClawFactory;
import frc.robot.subsystems.dualIntake.CommandDualIntakeFactory;
import frc.robot.subsystems.elevator.CommandElevatorFactory;
import frc.robot.subsystems.hook.CommandHookFactory;
import frc.robot.subsystems.swerveDrive.CommandSwerveFactory;
import frc.robot.subsystems.winch.CommandWinchFactory;
import frc.robot.subsystems.wrist.CommandWristFactory;

public class DriverRoutines {

    private final CommandXboxController driverController;
    private final CommandAlgaeClawFactory commandAlgaeClawFactory;
    private final CommandArmFactory commandArmFactory;
    private final CommandCoralClawFactory commandCoralClawFactory;
    private final CommandDualIntakeFactory commandDualIntakeFactory;
    private final CommandElevatorFactory commandElevatorFactory;
    private final CommandHookFactory commandHookFactory;
    private final CommandSwerveFactory commandSwerveFactory;
    private final CommandWinchFactory commandWinchFactory;
    private final CommandWristFactory commandWristFactory;
    private final SubsystemCommandFactories subsystemCommandFactories;

    public DriverRoutines(CommandXboxController driverController,
                            CommandAlgaeClawFactory commandAlgaeClawFactory,
                            CommandArmFactory commandArmFactory,
                            CommandCoralClawFactory commandCoralClawFactory,
                            CommandDualIntakeFactory commandDualIntakeFactory,
                            CommandElevatorFactory commandElevatorFactory,
                            CommandHookFactory commandHookFactory,
                            CommandSwerveFactory commandSwerveFactory,
                            CommandWinchFactory commandWinchFactory,
                            CommandWristFactory commandWristFactory,
                            SubsystemCommandFactories subsystemCommandFactories) {
        this.driverController = driverController;
        this.commandAlgaeClawFactory = commandAlgaeClawFactory;
        this.commandArmFactory = commandArmFactory;
        this.commandCoralClawFactory = commandCoralClawFactory;
        this.commandDualIntakeFactory = commandDualIntakeFactory;
        this.commandElevatorFactory = commandElevatorFactory;
        this.commandHookFactory = commandHookFactory;
        this.commandSwerveFactory = commandSwerveFactory;
        this.commandWinchFactory = commandWinchFactory;
        this.commandWristFactory = commandWristFactory;
        this.subsystemCommandFactories = subsystemCommandFactories;
    }
}
