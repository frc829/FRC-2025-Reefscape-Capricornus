package digilib.winch;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface WinchRequest {

    void apply(Winch winch);

    class DutyCycle implements WinchRequest {
        private final MutDimensionless dutyCycle = Percent.mutable(0.0);

        @Override
        public void apply(Winch winch) {
            winch.setDutyCycle(dutyCycle);
        }

        public DutyCycle withDutyCycle(Dimensionless dutyCycle) {
            this.dutyCycle.mut_replace(dutyCycle);
            return this;
        }
    }

    class Idle implements WinchRequest {
        @Override
        public void apply(Winch winch) {
            winch.setIdle();
        }
    }
}
