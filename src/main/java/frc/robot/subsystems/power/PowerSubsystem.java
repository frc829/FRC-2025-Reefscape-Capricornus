package frc.robot.subsystems.power;

import digilib.power.Power;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PowerSubsystem implements Subsystem {
    private final Power power;

    public PowerSubsystem(Power power) {
        this.power = power;
    }

    public Command clearFaults() {
        return runOnce(power::clearStickyFaults)
                .withName(String.format("%s: Clear Faults", getName()));
    }

    @Override
    public void periodic() {
        power.update();
    }
}
