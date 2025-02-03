package frc.robot.mechanisms.winch;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface WinchRequest {

    public void apply(Winch winch);

    public class Hold implements WinchRequest {
        @Override
        public void apply(Winch winch) {
            winch.setDutyCycle(Percent.of(0.0));
        }
    }

    public class DutyCycle implements WinchRequest {
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
}
