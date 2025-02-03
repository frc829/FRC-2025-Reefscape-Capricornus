package frc.robot.mechanisms.claws;

import static frc.robot.mechanisms.claws.ClawState.ClawValue.*;

public interface ClawRequest {

    public void apply(Claw claw);

    public class Open implements ClawRequest {
        @Override
        public void apply(Claw claw) {
            claw.setValue(OPEN);
        }
    }

    public class Close implements ClawRequest {
        @Override
        public void apply(Claw claw) {
            claw.setValue(CLOSED);
        }
    }


}
