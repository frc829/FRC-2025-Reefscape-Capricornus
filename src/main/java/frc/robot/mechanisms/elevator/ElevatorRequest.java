package frc.robot.mechanisms.elevator;

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


}
