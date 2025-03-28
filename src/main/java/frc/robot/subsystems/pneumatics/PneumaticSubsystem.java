package frc.robot.subsystems.pneumatics;

import digilib.pneumatics.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PneumaticSubsystem implements Subsystem {
    private final Pneumatics pneumatics;

    public PneumaticSubsystem(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
    }

    public Command clearFaults() {
        return runOnce(pneumatics::clearStickyFaults)
                .withName(String.format("%s: Clear Faults", getName()));
    }

    public Command turnCompressorOn() {
        return runOnce(pneumatics::turnOnCompressor)
                .withName(String.format("%s: Turn On Compressor", getName()));
    }

    public Command turnCompressorOff() {
        return runOnce(pneumatics::turnOffCompressor)
                .withName(String.format("%s: Turn Off Compressor", getName()));
    }

    @Override
    public void periodic() {
        pneumatics.update();
    }
}
