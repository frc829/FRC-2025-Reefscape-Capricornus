package frc.robot.subsystems.pneumatics;

import digilib.pneumatics.PneumaticsRequest;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandPneumaticsFactory {

    private final CommandPneumatics commandPneumatics;

    public CommandPneumaticsFactory(CommandPneumatics commandPneumatics) {
        this.commandPneumatics = commandPneumatics;
        commandPneumatics.register();
    }

    public Command clearFaults(){
        PneumaticsRequest.ClearStickyFaults request = new PneumaticsRequest.ClearStickyFaults();
        return commandPneumatics.applyRequestOnce(() -> request).withName("Pneumatics: Clear Faults");
    }

    public Command turnCompressorOn(){
        PneumaticsRequest.TurnOnCompressor request = new PneumaticsRequest.TurnOnCompressor();
        return commandPneumatics.applyRequestOnce(() -> request).withName("Pneumatics: Turn On Compressor");
    }

    public Command turnCompressorOff(){
        PneumaticsRequest.TurnOffCompressor request = new PneumaticsRequest.TurnOffCompressor();
        return commandPneumatics.applyRequestOnce(() -> request).withName("Pneumatics: Turn Off Compressor");
    }
}
