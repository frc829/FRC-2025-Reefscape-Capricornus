package frc.robot.commandFactories;

import frc.robot.subsystems.algaeClaw.CommandAlgaeClawFactory;
import frc.robot.subsystems.arm.CommandArmFactory;
import frc.robot.subsystems.coralClaw.CommandCoralClawFactory;
import frc.robot.subsystems.dualIntake.CommandDualIntakeFactory;
import frc.robot.subsystems.elevator.CommandElevatorFactory;
import frc.robot.subsystems.hook.CommandHookFactory;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveFactory;
import frc.robot.subsystems.winch.CommandWinchFactory;
import frc.robot.subsystems.wrist.CommandWristFactory;

public class SubsystemCommandFactories {
    public final CommandAlgaeClawFactory commandAlgaeClawFactory;
    public final CommandArmFactory commandArmFactory;
    public final CommandCoralClawFactory commandCoralClawFactory;
    public final CommandDualIntakeFactory commandDualIntakeFactory;
    public final CommandElevatorFactory commandElevatorFactory;
    public final CommandHookFactory commandHookFactory;
    public final CommandSwerveDriveFactory commandSwerveDriveFactory;
    public final CommandWinchFactory commandWinchFactory;
    public final CommandWristFactory commandWristFactory;

    public SubsystemCommandFactories(
            CommandAlgaeClawFactory commandAlgaeClawFactory,
            CommandArmFactory commandArmFactory,
            CommandCoralClawFactory commandCoralClawFactory,
            CommandDualIntakeFactory commandDualIntakeFactory,
            CommandElevatorFactory commandElevatorFactory,
            CommandHookFactory commandHookFactory,
            CommandSwerveDriveFactory commandSwerveDriveFactory,
            CommandWinchFactory commandWinchFactory,
            CommandWristFactory commandWristFactory) {
        this.commandAlgaeClawFactory = commandAlgaeClawFactory;
        this.commandArmFactory = commandArmFactory;
        this.commandCoralClawFactory = commandCoralClawFactory;
        this.commandDualIntakeFactory = commandDualIntakeFactory;
        this.commandElevatorFactory = commandElevatorFactory;
        this.commandHookFactory = commandHookFactory;
        this.commandSwerveDriveFactory = commandSwerveDriveFactory;
        this.commandWinchFactory = commandWinchFactory;
        this.commandWristFactory = commandWristFactory;
    }
}
