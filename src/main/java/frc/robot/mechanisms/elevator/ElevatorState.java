package frc.robot.mechanisms.elevator;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class ElevatorState implements Cloneable {

    private final MutDistance position = Meters.mutable(0.0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);

    public Distance getPosition() {
        return position;
    }

    public LinearVelocity getVelocity() {
        return velocity;
    }

    public ElevatorState withPosition(Distance position){
        this.position.mut_replace(position);
        return this;
    }

    public ElevatorState withVelocity(LinearVelocity velocity){
        this.velocity.mut_replace(velocity);
        return this;
    }

    public ElevatorState withTimestamp(Time timestamp){
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public ElevatorState withElevatorState(ElevatorState elevatorState){
        this.position.mut_replace(elevatorState.position);
        this.velocity.mut_replace(elevatorState.velocity);
        this.timestamp.mut_replace(elevatorState.timestamp);
        return this;
    }

    @Override
    public ElevatorState clone() {
        try {
            ElevatorState toReturn =  (ElevatorState) super.clone();
            toReturn.position.mut_replace(position);
            toReturn.velocity.mut_replace(velocity);
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
