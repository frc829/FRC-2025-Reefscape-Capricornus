package frc.robot.subsystems.algaeClaw;

import digilib.claws.ClawRequest;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandAlgaeClawFactory {
    private final CommandAlgaeClaw commandAlgaeClaw;

    public CommandAlgaeClawFactory(CommandAlgaeClaw commandAlgaeClaw) {
        this.commandAlgaeClaw = commandAlgaeClaw;
        commandAlgaeClaw.register();
    }

    public Command open(){
        ClawRequest.Open request = new ClawRequest.Open();
        return commandAlgaeClaw.applyRequestOnce(() -> request).withName("Algae: Open");
    }

    public Command close(){
        ClawRequest.Close request = new ClawRequest.Close();
        return commandAlgaeClaw.applyRequestOnce(() -> request).withName("Algae: Close");
    }

    public Command toggle(){
        ClawRequest.Toggle request = new ClawRequest.Toggle();
        return commandAlgaeClaw.applyRequestOnce(() -> request).withName("Algae: Toggle");
    }
}
