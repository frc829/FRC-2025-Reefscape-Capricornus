package frc.robot.mechanisms.elevator;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface ElevatorRequest {

    public void apply(Elevator elevator);

    public class Hold implements ElevatorRequest {
        @Override
        public void apply(Elevator elevator) {
            elevator.setHold();
        }
    }

    public class Position implements ElevatorRequest {
        private final MutDistance position = Meters.mutable(0.0);
        private final Distance minHeight;
        private final Distance maxHeight;

        public Position(Distance minHeight, Distance maxHeight) {
            this.minHeight = minHeight;
            this.maxHeight = maxHeight;
        }

        @Override
        public void apply(Elevator elevator) {
            if(position.lte(maxHeight) && position.gte(minHeight)) {
                elevator.setPosition(position);
            }else{
                elevator.setVelocity(MetersPerSecond.of(0.0));
            }
        }

        public Position withPosition(Distance position){
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements ElevatorRequest {
        private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
        private final Distance minHeight;
        private final Distance maxHeight;

        public Velocity(Distance minHeight, Distance maxHeight) {
            this.minHeight = minHeight;
            this.maxHeight = maxHeight;
        }
        @Override
        public void apply(Elevator elevator) {
            ElevatorState elevatorState = elevator.getState();
            if(elevatorState.getPosition().lte(maxHeight) && elevatorState.getPosition().gte(minHeight)){
                elevator.setVelocity(velocity);
            }else{
                elevator.setVelocity(MetersPerSecond.of(0.0));
            }
        }

        public Velocity withVelocity(LinearVelocity velocity){
            this.velocity.mut_replace(velocity);
            return this;
        }
    }


}
