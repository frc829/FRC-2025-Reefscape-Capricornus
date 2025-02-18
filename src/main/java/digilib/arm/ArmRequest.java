package digilib.arm;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface ArmRequest {

    void apply(Arm arm);

    class Position implements ArmRequest {
        private final MutAngle position = Radians.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            ArmState armState = arm.getState();
            if (position.lte(arm.getMaxAngle()) && position.gte(arm.getMinAngle())) {
                arm.setPosition(position);
            } else if (armState.getAngle().gte(arm.getMaxAngle()) && position.lte(arm.getMaxAngle())) {
                arm.setPosition(position);
            } else if (armState.getAngle().lte(arm.getMinAngle()) && position.gte(arm.getMinAngle())) {
                arm.setPosition(position);
            } else {
                arm.setPosition(armState.getAngle());
            }
        }

        public Position withPosition(Angle angle) {
            this.position.mut_replace(angle);
            return this;
        }
    }

    class Velocity implements ArmRequest {
        private final MutDimensionless maxPercent = Value.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            ArmState state = arm.getState();
            if (state.getAngle().gte(arm.getMaxAngle()) && maxPercent.gte(Value.of(0.0))) {
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            } else if (state.getAngle().lte(arm.getMinAngle()) && maxPercent.lte(Value.of(0.0))) {
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            }
            arm.setVelocity(maxPercent);
        }

        public Velocity withVelocity(Dimensionless maxPercent) {
            this.maxPercent.mut_replace(maxPercent);
            return this;
        }
    }

    class VoltageRequest implements ArmRequest {
        private final MutVoltage voltage = Volts.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            arm.setVoltage(voltage);
        }

        public VoltageRequest withVoltage(Voltage voltage) {
            this.voltage.mut_replace(voltage);
            return this;
        }
    }
}
