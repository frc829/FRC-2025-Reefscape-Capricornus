package digilib.power;

public interface PowerRequest {

    void apply(Power power);

    class ClearStickyFaults implements PowerRequest {

        @Override
        public void apply(Power power) {
            power.clearStickyFaults();
        }
    }
}
