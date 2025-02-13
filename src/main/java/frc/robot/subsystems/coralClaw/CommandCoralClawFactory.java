package frc.robot.subsystems.coralClaw;

import digilib.claws.ClawRequest;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandCoralClawFactory {
    private final CommandCoralClaw commandCoralClaw;

    public CommandCoralClawFactory(CommandCoralClaw commandCoralClaw) {
        this.commandCoralClaw = commandCoralClaw;
        commandCoralClaw.register();

    }

    public Command open(){
        ClawRequest.Open request = new ClawRequest.Open();
        return commandCoralClaw.applyRequestOnce(() -> request).withName("Coral: Open");
    }

    public Command close(){
        ClawRequest.Close request = new ClawRequest.Close();
        return commandCoralClaw.applyRequestOnce(() -> request).withName("Coral: Close");
    }

    public Command toggle(){
        ClawRequest.Toggle request = new ClawRequest.Toggle();
        return commandCoralClaw.applyRequestOnce(() -> request).withName("Coral: Toggle");
    }




}
