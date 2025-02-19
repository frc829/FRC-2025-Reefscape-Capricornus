package digilib.elevator;

import digilib.arm.ArmRequest;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface ElevatorRequest {

    void apply(Elevator elevator);

    class Position implements ElevatorRequest {
        private final MutDistance height = Meters.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            ElevatorState state = elevator.getState();
            if (height.gt(elevator.getMaxHeight())) {
                height.mut_replace(state.getHeight());
            } else if (height.lt(elevator.getMinHeight())) {
                height.mut_replace(state.getHeight());
            }
            elevator.setHeight(height);
        }

        public ElevatorRequest.Position withPosition(Distance height) {
            this.height.mut_replace(height);
            return this;
        }
    }

    class Velocity implements ElevatorRequest {
        private final MutDimensionless maxPercent = Value.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            ElevatorState state = elevator.getState();
            if (state.getHeight().gte(elevator.getMaxHeight()) && maxPercent.gte(Value.of(0.0))) {
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            } else if (state.getHeight().lte(elevator.getMinHeight()) && maxPercent.lte(Value.of(0.0))) {
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            }
            elevator.setVelocity(maxPercent);
        }

        public Velocity withVelocity(Dimensionless maxPercent) {
            this.maxPercent.mut_replace(maxPercent);
            return this;
        }
    }

    class VoltageRequest implements ElevatorRequest {
        private final MutVoltage voltage = Volts.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            elevator.setVoltage(voltage);
        }

        public VoltageRequest withVoltage(Voltage voltage) {
            this.voltage.mut_replace(voltage);
            return this;
        }
    }
}
