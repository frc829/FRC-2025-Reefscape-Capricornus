package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandAlgaeClaw;

public class CommandAlgaeClawFactory {
    private final CommandAlgaeClaw commandAlgaeClaw;

    public CommandAlgaeClawFactory(CommandAlgaeClaw commandAlgaeClaw) {
        this.commandAlgaeClaw = commandAlgaeClaw;
    }

    public Command open(){
        // TODO: create a ClawRequest.Open called request and assign it to a new ClawRequest.Open();
        // TODO: return commandAlgaeClaw.applyRequest(() -> request).withName("OPEN");
        return null; // TODO: remove this when done.
    }

    public Command close(){
        // TODO:  same as open but with close :)
        return null; // TODO: remove this when done.
    }




}
