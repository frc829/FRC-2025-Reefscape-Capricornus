package digilib.claws;

public enum ClawValue {
    OPEN,
    CLOSED;

    public ClawValue opposite() {
        return switch (this) {
            case OPEN -> CLOSED;
            case CLOSED -> OPEN;
        };
    }
}
