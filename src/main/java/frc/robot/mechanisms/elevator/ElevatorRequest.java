package frc.robot.mechanisms.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public interface ElevatorRequest {

    public boolean apply(ElevatorControlParameters parameters, Elevator elevator);

    public class Hold implements ElevatorRequest {
        @Override
        public boolean apply(ElevatorControlParameters parameters, Elevator elevator) {
            return false;
        }
    }

    public class FreeFall implements ElevatorRequest {
        @Override
        public boolean apply(ElevatorControlParameters parameters, Elevator elevator) {
            return false;
        }
    }

    public class Position implements ElevatorRequest {
        private final MutDistance position = Meters.mutable(0.0);

        @Override
        public boolean apply(ElevatorControlParameters parameters, Elevator elevator) {
            return false;
        }

        public Position withPosition(Distance position){
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements ElevatorRequest {
        private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);

        @Override
        public boolean apply(ElevatorControlParameters parameters, Elevator elevator) {
            return false;
        }

        public Velocity withVelocity(LinearVelocity velocity){
            this.velocity.mut_replace(velocity);
            return this;
        }
    }


}
