package digilib.wrist;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class WristState implements Cloneable {

    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final MutVoltage voltage = Volts.mutable(0.0);

    public Angle getPosition() {
        return position;
    }

    public AngularVelocity getVelocity() {
        return velocity;
    }

    public WristState withPosition(Angle position) {
        this.position.mut_replace(position);
        return this;
    }

    public WristState withVelocity(AngularVelocity velocity) {
        this.velocity.mut_replace(velocity);
        return this;
    }

    public MutVoltage getVoltage() {
        return voltage;
    }

    public WristState withTimestamp(Time timestamp) {
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public WristState withVoltage(double voltage) {
        this.voltage.mut_setMagnitude(voltage);
        return this;
    }

    public WristState withWristState(WristState wristState) {
        this.position.mut_replace(wristState.position);
        this.velocity.mut_replace(wristState.velocity);
        this.timestamp.mut_replace(wristState.timestamp);
        return this;
    }

    @Override
    public WristState clone() {
        try {
            WristState toReturn = (WristState) super.clone();
            toReturn.position.mut_replace(position);
            toReturn.velocity.mut_replace(velocity);
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
