package digilib.pneumatics;

public interface PneumaticsRequest {

    void apply(Pneumatics pneumatics);

    class ClearStickyFaults implements PneumaticsRequest {

        @Override
        public void apply(Pneumatics pneumatics) {
            pneumatics.clearStickyFaults();
        }
    }

    class TurnOnCompressor implements PneumaticsRequest {

        @Override
        public void apply(Pneumatics pneumatics) {
            pneumatics.turnOnCompressor();
        }
    }

    class TurnOffCompressor implements PneumaticsRequest {

        @Override
        public void apply(Pneumatics pneumatics) {
            pneumatics.turnOffCompressor();
        }
    }
}
