package frc.robot.commandFactories;

import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    public final CommandAlgaeClawFactory algae;
    public final CommandArmFactory arm;
    public final CommandCoralClawFactory coral;
    public final CommandDualIntakeFactory intake;
    public final CommandElevatorFactory elevator;
    public final CommandHookFactory hook;
    public final CommandSwerveDriveFactory swerve;
    public final CommandWinchFactory winch;
    public final CommandWristFactory wrist;

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
        this.algae = commandAlgaeClawFactory;
        this.arm = commandArmFactory;
        this.coral = commandCoralClawFactory;
        this.intake = commandDualIntakeFactory;
        this.elevator = commandElevatorFactory;
        this.hook = commandHookFactory;
        this.swerve = commandSwerveDriveFactory;
        this.winch = commandWinchFactory;
        this.wrist = commandWristFactory;
    }
}
