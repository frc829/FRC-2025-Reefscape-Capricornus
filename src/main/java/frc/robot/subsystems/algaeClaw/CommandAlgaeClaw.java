package frc.robot.subsystems.algaeClaw;

import digilib.claws.Claw;
import digilib.claws.ClawRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

public class CommandAlgaeClaw implements Subsystem {
    private final Claw claw;

    public CommandAlgaeClaw(Claw claw) {
        this.claw = claw;
    }

    public Command applyRequest(Supplier<ClawRequest> requestSupplier) {
        return runOnce(() -> claw.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        claw.update();
    }


}
