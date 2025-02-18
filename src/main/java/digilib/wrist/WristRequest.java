package digilib.wrist;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface WristRequest {

    void apply(Wrist wrist);

    class Position implements WristRequest {
        private final MutAngle angle = Radians.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            WristState state = wrist.getState();
            if (angle.gt(wrist.getMaxAngle())) {
                angle.mut_replace(state.getAngle());
            } else if (angle.lt(wrist.getMinAngle())) {
                angle.mut_replace(state.getAngle());
            }
            wrist.setPosition(angle);
        }

        public WristRequest.Position withAngle(Angle angle) {
            this.angle.mut_replace(angle);
            return this;
        }
    }

    class Velocity implements WristRequest {
        private final MutDimensionless maxPercent = Value.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            WristState state = wrist.getState();
            if (state.getAngle().gte(wrist.getMaxAngle()) && maxPercent.gte(Value.of(0.0))) {
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            } else if (state.getAngle().lte(wrist.getMinAngle()) && maxPercent.lte(Value.of(0.0))) {
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            }
            wrist.setVelocity(maxPercent);
        }

        public WristRequest.Velocity withVelocity(Dimensionless maxPercent) {
            this.maxPercent.mut_replace(maxPercent);
            return this;
        }
    }

    class VoltageRequest implements WristRequest {
        private final MutVoltage voltage = Volts.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            wrist.setVoltage(voltage);
        }

        public VoltageRequest withVoltage(Voltage voltage) {
            this.voltage.mut_replace(voltage);
            return this;
        }
    }
}
