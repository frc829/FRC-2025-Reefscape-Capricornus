package frc.robot.subsystems.coralClaw;

import digilib.claws.ClawRequest;
import digilib.claws.ClawState;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandCoralClawFactory {
    private final CommandCoralClaw commandCoralClaw;

    public CommandCoralClawFactory(CommandCoralClaw commandCoralClaw) {
        this.commandCoralClaw = commandCoralClaw;
    }

    public Command setClawValue(ClawState.ClawValue clawValue){
        ClawRequest.SetClaw request = new ClawRequest.SetClaw(clawValue);
        return commandCoralClaw.applyRequest(() -> request).withName(clawValue.name());
    }




}
