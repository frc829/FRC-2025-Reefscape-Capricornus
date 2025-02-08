package digilib.claws;

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
}
