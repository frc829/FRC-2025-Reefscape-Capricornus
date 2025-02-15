package frc.robot.commandFactories;

import frc.robot.subsystems.algaeClaw.CommandAlgaeClawFactory;
import frc.robot.subsystems.arm.CommandArm;
import frc.robot.subsystems.coralClaw.CommandCoralClawFactory;
import frc.robot.subsystems.dualIntake.CommandDualIntakeFactory;
import frc.robot.subsystems.elevator.CommandElevator;
import frc.robot.subsystems.pneumatics.CommandPneumaticsFactory;
import frc.robot.subsystems.power.CommandPowerFactory;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveFactory;
import frc.robot.subsystems.winch.CommandWinchFactory;
import frc.robot.subsystems.wrist.CommandWrist;

public class SubsystemCommandFactories {
    public final CommandAlgaeClawFactory algae;
    public final CommandArm arm;
    public final CommandCoralClawFactory coral;
    public final CommandDualIntakeFactory intake;
    public final CommandElevator elevator;
    public final CommandPneumaticsFactory commandPneumaticsFactory;
    public final CommandPowerFactory commandPowerFactory;
    public final CommandSwerveDriveFactory swerve;
    public final CommandWinchFactory winch;
    public final CommandWrist wrist;

    public SubsystemCommandFactories(
            CommandAlgaeClawFactory commandAlgaeClawFactory,
            CommandArm arm,
            CommandCoralClawFactory commandCoralClawFactory,
            CommandDualIntakeFactory commandDualIntakeFactory,
            CommandElevator elevator,
            CommandPneumaticsFactory commandPneumaticsFactory,
            CommandPowerFactory commandPowerFactory,
            CommandSwerveDriveFactory commandSwerveDriveFactory,
            CommandWinchFactory commandWinchFactory,
            CommandWrist wrist) {
        this.algae = commandAlgaeClawFactory;
        this.arm = arm;
        this.coral = commandCoralClawFactory;
        this.intake = commandDualIntakeFactory;
        this.elevator = elevator;
        this.commandPneumaticsFactory = commandPneumaticsFactory;
        this.commandPowerFactory = commandPowerFactory;
        this.swerve = commandSwerveDriveFactory;
        this.winch = commandWinchFactory;
        this.wrist = wrist;
    }
}
