package digilib.intakeWheel;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class IntakeWheelState {

    private final Distance wheelRadius;
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutLinearVelocity linearVelocity = MetersPerSecond.mutable(0.0);
    private final MutVoltage voltage = Volts.mutable(0.0);
    private final MutCurrent current = Amps.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);

    public IntakeWheelState(Distance wheelRadius) {
        this.wheelRadius = wheelRadius;
    }

    public AngularVelocity getAngularVelocity() {
        return velocity;
    }

    public LinearVelocity getVelocity() {
        return linearVelocity;
    }

    public Voltage getVoltage() {
        return voltage;
    }

    public Current getCurrent() {
        return current;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public IntakeWheelState withAngularVelocity(double radiansPerSecond) {
        this.velocity.mut_setBaseUnitMagnitude(radiansPerSecond);
        this.linearVelocity.mut_setBaseUnitMagnitude(radiansPerSecond * wheelRadius.baseUnitMagnitude());
        return this;
    }

    public IntakeWheelState withVoltage(double volts) {
        this.voltage.mut_setBaseUnitMagnitude(volts);
        return this;
    }

    public IntakeWheelState withCurrent(double amps) {
        this.current.mut_setBaseUnitMagnitude(amps);
        return this;
    }

    public IntakeWheelState withTimestamp(double seconds) {
        this.timestamp.mut_setBaseUnitMagnitude(seconds);
        return this;
    }
}
