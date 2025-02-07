package frc.robot.subsystems.algaeClaw;

import digilib.claws.Claw;
import digilib.claws.ClawRequest;
import digilib.claws.ClawState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

public class CommandAlgaeClaw implements Subsystem {
    private final Claw claw;
    public final Trigger isOpen;
    public final Trigger isClosed;

    public CommandAlgaeClaw(Claw claw) {
        this.claw = claw;
        isOpen = new Trigger(() -> claw.getState().getClawValue() == ClawState.ClawValue.OPEN);
        isClosed = new Trigger(() -> claw.getState().getClawValue() == ClawState.ClawValue.CLOSED);
    }

    public Command applyRequest(Supplier<ClawRequest> requestSupplier) {
        return runOnce(() -> claw.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        claw.update();
    }


}
