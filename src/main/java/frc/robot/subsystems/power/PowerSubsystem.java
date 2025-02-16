package frc.robot.subsystems.power;

import digilib.power.Power;
import digilib.power.PowerRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

public class PowerSubsystem implements Subsystem {
    private final Power power;

    public PowerSubsystem(Power power) {
        this.power = power;
    }

    public Command applyRequestOnce(Supplier<PowerRequest> requestSupplier) {
        return runOnce(() -> power.setControl(requestSupplier.get()));
    }

    public Command clearFaults() {
        PowerRequest.ClearStickyFaults request = new PowerRequest.ClearStickyFaults();
        return applyRequestOnce(() -> request)
                .withName(String.format("%s: Clear Faults", getName()));
    }

    @Override
    public void periodic() {
        power.update();
    }
}
