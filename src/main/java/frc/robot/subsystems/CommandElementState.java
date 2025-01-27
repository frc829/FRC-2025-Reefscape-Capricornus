package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.twostate.TwoStateValue;

public class CommandElementState implements Subsystem {

    public enum ElementState{
        HOLD(TwoStateValue.STATE_0),
        RELEASE(TwoStateValue.STATE_1);

        private TwoStateValue value;

        ElementState(TwoStateValue value){
            this.value = value;
        }

        public TwoStateValue getValue() {
            return value;
        }
    }

}
