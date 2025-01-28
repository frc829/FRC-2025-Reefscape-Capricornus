package frc.robot.mechanisms.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public interface ElevatorRequest {

    public void apply(ElevatorControlParameters parameters, Elevator elevator);

    public class Hold implements ElevatorRequest {
        @Override
        public void apply(ElevatorControlParameters parameters, Elevator elevator) {
            elevator.setHold();
        }
    }

    public class Position implements ElevatorRequest {
        private final MutDistance position = Meters.mutable(0.0);

        @Override
        public void apply(ElevatorControlParameters parameters, Elevator elevator) {
            double maxHeightMeters = parameters.getMaxHeight().baseUnitMagnitude();
            double minHeightMeters = parameters.getMinHeight().baseUnitMagnitude();
            if(position.lte(parameters.getMaxHeight()) && position.gte(parameters.getMinHeight())) {
                elevator.setPosition(position);
            }else{
                elevator.setPosition(parameters.getCurrentState().getPosition());
            }
        }

        public Position withPosition(Distance position){
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements ElevatorRequest {
        private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);

        @Override
        public void apply(ElevatorControlParameters parameters, Elevator elevator) {
            if(parameters.getCurrentState().getPosition().lte(parameters.getMaxHeight()) && parameters.getCurrentState().getPosition().gte(parameters.getMinHeight())){
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
