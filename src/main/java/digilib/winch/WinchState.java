package digilib.winch;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class WinchState implements Cloneable {

    private final MutDimensionless dutyCycle = Percent.mutable(0.0);
    private final MutDistance position = Meters.mutable(0.0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);

    public Dimensionless getDutyCycle() {
        return dutyCycle;
    }

    public Distance getPosition() {
        return position;
    }

    public LinearVelocity getVelocity() {
        return velocity;
    }

    public WinchState withDutyCycle(MutDimensionless dutyCycle){
        this.dutyCycle.mut_replace(dutyCycle);
        return this;
    }

    public WinchState withTimestamp(Time timestamp){
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public WinchState withPosition(Distance position){
        this.position.mut_replace(position);
        return this;
    }

    public WinchState withVelocity(LinearVelocity velocity){
        this.velocity.mut_replace(velocity);
        return this;
    }

    public WinchState withWinchState(WinchState winchState){
        this.dutyCycle.mut_replace(winchState.dutyCycle);
        this.timestamp.mut_replace(winchState.timestamp);
        return this;
    }

    @Override
    public WinchState clone() {
        try {
            WinchState toReturn =  (WinchState) super.clone();
            toReturn.dutyCycle.mut_replace(dutyCycle);
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
