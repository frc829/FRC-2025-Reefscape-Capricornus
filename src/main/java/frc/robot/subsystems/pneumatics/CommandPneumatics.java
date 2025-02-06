package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

public class CommandPneumatics implements Subsystem {
    private final PneumaticsBase pneumatics;

    public CommandPneumatics(PneumaticsBase pneumatics) {
        this.pneumatics = pneumatics;
    }

    public Command applyRequest(Supplier<PneumaticsRequest> requestSupplier) {
        // return run(() -> pneumatics.setControl(requestSupplier.get()));
        return null; // TODO: will do later.
    }

    @Override
    public void periodic() {
        // TODO: will do later.
    }
}
