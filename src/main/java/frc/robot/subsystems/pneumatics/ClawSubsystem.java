package frc.robot.subsystems.pneumatics;

import digilib.claws.Claw;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static digilib.claws.ClawState.*;

public class ClawSubsystem implements Subsystem {
    private final Claw claw;
    public final Trigger isOpen;
    public final Trigger isClosed;

    public ClawSubsystem(Claw claw) {
        this.claw = claw;
        isOpen = new Trigger(() -> claw.getState().getClawValue() == ClawValue.OPEN);
        isClosed = new Trigger(() -> claw.getState().getClawValue() == ClawValue.CLOSED);
    }

    public Command toClawValue(ClawValue clawValue) {
        return runOnce(() -> claw.setValue(clawValue));
    }

    public Command toggle(){
        return runOnce(claw::toggle);
    }

    @Override
    public void periodic() {
        claw.update();
    }
}
