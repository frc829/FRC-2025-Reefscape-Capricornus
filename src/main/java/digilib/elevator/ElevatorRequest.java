package digilib.elevator;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public interface ElevatorRequest {

    void apply(Elevator elevator);

    class Position implements ElevatorRequest {
        private final MutDistance height = Meters.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            ElevatorState state = elevator.getState();
            if (state.getHeight().gte(elevator.getMaxHeight()) && height.gt(elevator.getMaxHeight())) {
                elevator.setHeight(height.mut_replace(elevator.getMaxHeight()));
            } else if (state.getHeight().lte(elevator.getMinHeight()) && height.lt(elevator.getMinHeight())) {
                elevator.setHeight(height.mut_replace(elevator.getMinHeight()));
            } else {
                elevator.setHeight(height);
            }
            SmartDashboard.putNumber("Elevator Setpoint", height.in(Meters));
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
            if (state.getHeight().gte(elevator.getMaxHeight()) && maxPercent.gt(Value.of(0.0))) {
                elevator.setHeight(elevator.getMaxHeight());
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            } else if (state.getHeight().lte(elevator.getMinHeight()) && maxPercent.lt(Value.of(0.0))) {
                elevator.setHeight(elevator.getMinHeight());
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            } else{
                elevator.setVelocity(maxPercent);
            }
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
