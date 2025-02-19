package frc.robot.subsystems.power;

import digilib.power.PowerRequest;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandPowerFactory {

    private final CommandPower commandPower;

    public CommandPowerFactory(CommandPower commandPower) {
        this.commandPower = commandPower;
        commandPower.register();
    }

    public Command clearFaults(){
        PowerRequest.ClearStickyFaults request = new PowerRequest.ClearStickyFaults();
        return commandPower.applyRequestOnce(() -> request).withName("Power: Clear Faults");
    }
}
