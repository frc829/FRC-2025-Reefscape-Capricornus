package frc.robot.subsystems.pneumatics;

import digilib.pneumatics.Pneumatics;
import digilib.pneumatics.PneumaticsRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

public class PneumaticSubsystem implements Subsystem {
    private final Pneumatics pneumatics;
    public final Trigger compressorOn;
    public final Trigger compressorOff;

    public PneumaticSubsystem(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
        compressorOn = new Trigger(() -> pneumatics.getState().isCompressorOn());
        compressorOff = new Trigger(() -> !pneumatics.getState().isCompressorOn());
    }

    public Command applyRequestOnce(Supplier<PneumaticsRequest> requestSupplier) {
        return runOnce(() -> pneumatics.setControl(requestSupplier.get()));
    }

    public Command clearFaults() {
        PneumaticsRequest.ClearStickyFaults request = new PneumaticsRequest.ClearStickyFaults();
        return applyRequestOnce(() -> request)
                .withName(String.format("%s: Clear Faults", getName()));
    }

    public Command turnCompressorOn() {
        PneumaticsRequest.TurnOnCompressor request = new PneumaticsRequest.TurnOnCompressor();
        return applyRequestOnce(() -> request)
                .withName(String.format("%s: Turn On Compressor", getName()));
    }

    public Command turnCompressorOff() {
        PneumaticsRequest.TurnOffCompressor request = new PneumaticsRequest.TurnOffCompressor();
        return applyRequestOnce(() -> request)
                .withName(String.format("%s: Turn Off Compressor", getName()));
    }

    @Override
    public void periodic() {
        pneumatics.update();
    }
}
