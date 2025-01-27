package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.twostate.TwoStateValue;

public class CommandClawState implements Subsystem {

    public enum ClawState {
        ALGAE(TwoStateValue.STATE_0),
        CORAL(TwoStateValue.STATE_1),;

        private TwoStateValue value;

        ClawState(TwoStateValue value) {
            this.value = value;
        }

        public TwoStateValue getValue() {
            return value;
        }
    }

}
