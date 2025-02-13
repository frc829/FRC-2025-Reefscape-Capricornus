package digilib.claws;

import static digilib.claws.ClawValue.CLOSED;
import static digilib.claws.ClawValue.OPEN;

public interface ClawRequest {

    void apply(Claw claw);

    class Open implements ClawRequest {

        @Override
        public void apply(Claw claw) {
            claw.setValue(OPEN);
        }
    }

    class Close implements ClawRequest {

        @Override
        public void apply(Claw claw) {
            claw.setValue(CLOSED);
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
