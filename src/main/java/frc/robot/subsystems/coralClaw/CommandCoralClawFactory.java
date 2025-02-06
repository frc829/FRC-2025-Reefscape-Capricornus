package frc.robot.subsystems.coralClaw;

import digilib.claws.ClawRequest;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandCoralClawFactory {
    private final CommandCoralClaw commandCoralClaw;

    public CommandCoralClawFactory(CommandCoralClaw commandCoralClaw) {
        this.commandCoralClaw = commandCoralClaw;
    }

    public Command open(){
        ClawRequest.Open request = new ClawRequest.Open();
        return commandCoralClaw.applyRequest(() -> request).withName("OPEN");
    }

    public Command close(){
        ClawRequest.Close request = new ClawRequest.Close();
        return commandCoralClaw.applyRequest(() -> request).withName("CLOSE");
    }




}
