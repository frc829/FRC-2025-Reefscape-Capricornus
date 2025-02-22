package digilib.claws;

import static digilib.claws.ClawValue.CLOSED;

public interface ClawRequest {

    void apply(Claw claw);

    class SetValue implements ClawRequest {

        private ClawValue value = null;

        @Override
        public void apply(Claw claw) {
            claw.setValue(value);
        }

        public SetValue withClawValue(ClawValue value) {
            this.value = value;
            return this;
        }
    }

    class Toggle implements ClawRequest {

        public void apply(Claw claw) {
            ClawValue clawValue = claw.getState().getClawValue();
            if (clawValue != null) {
                claw.setValue(clawValue.opposite());
            } else {
                claw.setValue(CLOSED);
            }
        }
    }
}
