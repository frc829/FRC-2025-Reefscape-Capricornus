package frc.robot.subsystems.pneumatics;

import digilib.pneumatics.Pneumatics;
import digilib.pneumatics.PneumaticsRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

public class CommandPneumatics implements Subsystem {
    private final Pneumatics pneumatics;
    public final Trigger compressorOn;
    public final Trigger compressorOff;

    public CommandPneumatics(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
        compressorOn = new Trigger(() -> pneumatics.getState().isCompressorOn());
        compressorOff = new Trigger(() -> !pneumatics.getState().isCompressorOn());
    }

    public Command applyRequestOnce(Supplier<PneumaticsRequest> requestSupplier) {
        return runOnce(() -> pneumatics.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        pneumatics.update();
    }
}
