package digilib.claws;

import static digilib.claws.ClawState.ClawValue.CLOSED;

public interface ClawRequest {

    public void apply(Claw claw);

    public class SetClaw implements ClawRequest {

        private final ClawState.ClawValue clawValue;

        public SetClaw(ClawState.ClawValue clawValue) {
            this.clawValue = clawValue;
        }

        @Override
        public void apply(Claw claw) {
            claw.setValue(clawValue);
        }
    }

    public class Toggle implements ClawRequest {

        public void apply(Claw claw) {
            ClawState.ClawValue clawValue = claw.getState().getClawValue();
            if(clawValue != null){
                claw.setValue(clawValue.opposite());
            }
            else{
                claw.setValue(CLOSED);
            }
        }
    }
}
