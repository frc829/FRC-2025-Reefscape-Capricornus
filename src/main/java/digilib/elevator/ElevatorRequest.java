package digilib.elevator;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface ElevatorRequest {

    void apply(Elevator elevator);

    class Position implements ElevatorRequest {
        private final MutDistance position = Meters.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            if (elevator.isHoldEnabled()) {
                elevator.disableHold();
            }
            ElevatorState wristState = elevator.getState();
            if (position.lte(elevator.getMaxPosition()) && position.gte(elevator.getMinPosition())) {
                elevator.setPosition(position);
            } else if (wristState.getPosition().gte(elevator.getMaxPosition()) && position.lte(elevator.getMaxPosition())) {
                elevator.setPosition(position);
            } else if (wristState.getPosition().lte(elevator.getMinPosition()) && position.gte(elevator.getMinPosition())) {
                elevator.setPosition(position);
            } else {
                elevator.setVelocity(MetersPerSecond.of(0.0));
            }
        }

        public Position withPosition(double meters) {
            this.position.mut_setBaseUnitMagnitude(meters);
            return this;
        }
    }

    class Velocity implements ElevatorRequest {
        private final MutDimensionless maxVelocityValue = Value.mutable(0.0);
        private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            if (elevator.isHoldEnabled()) {
                elevator.disableHold();
            }
            ElevatorState state = elevator.getState();
            if (state.getPosition().lte(elevator.getMaxPosition()) && state.getPosition().gte(elevator.getMinPosition())) {
                velocity.mut_setMagnitude(maxVelocityValue.baseUnitMagnitude() * elevator.getMaxVelocity().baseUnitMagnitude());
            } else if (state.getPosition().gte(elevator.getMaxPosition()) && maxVelocityValue.baseUnitMagnitude() < 0.0) {
                velocity.mut_setMagnitude(maxVelocityValue.baseUnitMagnitude() * elevator.getMaxVelocity().baseUnitMagnitude());
            } else if (state.getPosition().lte(elevator.getMinPosition()) && maxVelocityValue.baseUnitMagnitude() > 0.0) {
                velocity.mut_setMagnitude(maxVelocityValue.baseUnitMagnitude() * elevator.getMaxVelocity().baseUnitMagnitude());
            } else {
                velocity.mut_setMagnitude(0.0);
            }
            elevator.setVelocity(velocity);
        }

        public Velocity withVelocity(double value) {
            this.maxVelocityValue.mut_setBaseUnitMagnitude(value);
            return this;
        }
    }

    class Hold implements ElevatorRequest {
        private final MutDistance holdPosition = Meters.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            boolean isHoldEnabled = elevator.isHoldEnabled();
            if (!isHoldEnabled) {
                elevator.enableHold();
                holdPosition.mut_replace(elevator.getState().getPosition());
            }
            elevator.setPosition(holdPosition);
        }
    }
}
