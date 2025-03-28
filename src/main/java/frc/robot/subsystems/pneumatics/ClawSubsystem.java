package frc.robot.subsystems.pneumatics;

import digilib.claws.Claw;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static digilib.claws.Claw.*;
import static digilib.claws.Claw.Value.CLOSED;

public class ClawSubsystem implements Subsystem {
    private final Claw claw;

    public ClawSubsystem(Claw claw) {
        this.claw = claw;
    }

    public Command toValue(Value value) {
        return runOnce(() -> claw.setValue(value));
    }

    public Command toggle(){
        return runOnce(claw::toggle);
    }

    @Override
    public void periodic() {
        if(RobotState.isDisabled()){
            claw.setValue(CLOSED);
        }
        claw.update();
    }
}
