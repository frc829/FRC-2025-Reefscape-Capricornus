package frc.robot.subsystems.pneumatics;

import digilib.claws.Claw;
import digilib.claws.ClawValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

public class ClawSubsystem implements Subsystem {
    private final Claw claw;
    public final Trigger isOpen;
    public final Trigger isClosed;

    public ClawSubsystem(Claw claw) {
        this.claw = claw;
        isOpen = new Trigger(() -> claw.getState().getClawValue() == ClawValue.OPEN);
        isClosed = new Trigger(() -> claw.getState().getClawValue() == ClawValue.CLOSED);
    }

    public Command applyRequestOnce(Supplier<ClawRequest> requestSupplier) {
        return runOnce(() -> claw.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        claw.update();
    }
}
