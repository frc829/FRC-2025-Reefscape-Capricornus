package digilib.claws;

public class ClawState {

    public enum ClawValue {
        OPEN,
        CLOSED,
        UNKNOWN;

        public ClawValue opposite() {
            return switch (this) {
                case OPEN -> CLOSED;
                case CLOSED -> OPEN;
                default -> UNKNOWN;
            };
        }
    }

    private ClawValue clawValue = null;

    public ClawValue getClawValue() {
        return clawValue;
    }

    public void setClawValue(ClawValue clawValue) {
        this.clawValue = clawValue;
    }
}
