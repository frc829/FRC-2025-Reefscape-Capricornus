package digilib.intakeWheel;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class IntakeWheelState implements Cloneable {

    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final Distance wheelRadius;
    private final MutLinearVelocity linearVelocity = MetersPerSecond.mutable(0.0);

    public IntakeWheelState(Distance wheelRadius) {
        this.wheelRadius = wheelRadius;
    }

    public LinearVelocity getVelocity() {
        return linearVelocity.mut_setBaseUnitMagnitude(wheelRadius.baseUnitMagnitude() * velocity.baseUnitMagnitude());
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public IntakeWheelState withVelocity(AngularVelocity velocity) {
        this.velocity.mut_replace(velocity);
        return this;
    }

    public IntakeWheelState withTimestamp(Time timestamp) {
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public IntakeWheelState withIntakeState(IntakeWheelState intakeWheelState) {
        this.velocity.mut_replace(intakeWheelState.velocity);
        this.timestamp.mut_replace(intakeWheelState.timestamp);
        return this;
    }

    @Override
    public IntakeWheelState clone() {
        try {
            IntakeWheelState toReturn = (IntakeWheelState) super.clone();
            toReturn.velocity.mut_replace(velocity);
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }


}
