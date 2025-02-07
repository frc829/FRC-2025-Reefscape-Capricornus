package frc.robot.subsystems.algaeClaw;

import digilib.claws.ClawRequest;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandAlgaeClawFactory {
    private final CommandAlgaeClaw commandAlgaeClaw;

    public CommandAlgaeClawFactory(CommandAlgaeClaw commandAlgaeClaw) {
        this.commandAlgaeClaw = commandAlgaeClaw;
    }

    public Command open(){
        ClawRequest.Open request = new ClawRequest.Open();
        return commandAlgaeClaw.applyRequest(() -> request).withName("OPEN");
    }

    public Command close(){
        ClawRequest.Close request = new ClawRequest.Close();
        return commandAlgaeClaw.applyRequest(() -> request).withName("CLOSE");
    }




}
