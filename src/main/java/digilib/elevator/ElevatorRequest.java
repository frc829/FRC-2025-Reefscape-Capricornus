package digilib.elevator;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public interface ElevatorRequest {

    public void apply(Elevator elevator);

    public class Hold implements ElevatorRequest {
        private final MutDistance holdPosition = Meters.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            boolean isHoldEnabled = elevator.isHoldEnabled();
            if (!isHoldEnabled) {
                elevator.enableHold();
                holdPosition.mut_replace(elevator.getState().getPosition());
            }
            SmartDashboard.putNumber("Hold Position [meters]", holdPosition.in(Meters));
            elevator.setPosition(holdPosition);
        }
    }

    public class Position implements ElevatorRequest {
        private final MutDistance position = Meters.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            if (elevator.isHoldEnabled()) {
                elevator.disableHold();
            }
            if (position.lte(elevator.getMaxPosition()) && position.gte(elevator.getMinPosition())) {
                elevator.setPosition(position);
            } else {
                elevator.setVelocity(MetersPerSecond.of(0.0));
            }
            SmartDashboard.putNumber("Position Setpoint [meters]", position.in(Meters));
        }

        public Position withPosition(Distance position) {
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements ElevatorRequest {
        private final MutDimensionless maxVelocityPercent = Percent.mutable(0.0);
        private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);

        @Override
        public void apply(Elevator elevator) {
            if (elevator.isHoldEnabled()) {
                elevator.disableHold();
            }
            ElevatorState elevatorState = elevator.getState();
            if (elevatorState.getPosition().lte(elevator.getMaxPosition()) && elevatorState.getPosition().gte(elevator.getMinPosition())) {
                velocity.mut_setMagnitude(maxVelocityPercent.baseUnitMagnitude() * elevator.getMaxVelocity().baseUnitMagnitude());
            } else {
                velocity.mut_setMagnitude(0.0);
            }
            elevator.setVelocity(velocity);

        }

        public Velocity withVelocity(Dimensionless maxVelocityPercent) {
            this.maxVelocityPercent.mut_replace(maxVelocityPercent);
            return this;
        }
    }
}
