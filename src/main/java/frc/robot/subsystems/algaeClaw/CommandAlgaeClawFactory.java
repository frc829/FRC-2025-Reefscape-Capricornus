package frc.robot.subsystems.algaeClaw;

import digilib.claws.ClawRequest;
import edu.wpi.first.wpilibj2.command.Command;

import static digilib.claws.ClawState.*;

public class CommandAlgaeClawFactory {
    private final CommandAlgaeClaw commandAlgaeClaw;

    public CommandAlgaeClawFactory(CommandAlgaeClaw commandAlgaeClaw) {
        this.commandAlgaeClaw = commandAlgaeClaw;
    }

    public Command setClawValue(ClawValue clawValue){
        ClawRequest.SetClaw request = new ClawRequest.SetClaw(clawValue);
        return commandAlgaeClaw.applyRequest(() -> request).withName(clawValue.name());
    }




}
