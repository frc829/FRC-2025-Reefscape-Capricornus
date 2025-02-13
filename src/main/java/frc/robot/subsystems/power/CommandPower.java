package frc.robot.subsystems.power;

import digilib.power.Power;
import digilib.power.PowerRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

public class CommandPower implements Subsystem {
    private final Power power;

    public CommandPower(Power power) {
        this.power = power;
    }

    public Command applyRequestOnce(Supplier<PowerRequest> requestSupplier) {
        return runOnce(() -> power.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        power.update();
    }
}
