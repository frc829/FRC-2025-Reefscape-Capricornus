package digilib.intakeWheel;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class IntakeWheelState {

    private final MutAngularVelocity angularVelocity = RadiansPerSecond.mutable(0.0);
    private final MutVoltage voltage = Volts.mutable(0.0);
    private final MutCurrent current = Amps.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);

    public AngularVelocity getAngularVelocity() {
        return angularVelocity;
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

    public void setAngularVelocity(double radiansPerSecond) {
        this.angularVelocity.mut_setBaseUnitMagnitude(radiansPerSecond);
    }

    public void setAngularVelocity(AngularVelocity angularVelocity) {
        this.angularVelocity.mut_replace(angularVelocity);
    }

    public void setVoltage(double volts) {
        this.voltage.mut_setBaseUnitMagnitude(volts);
    }

    public void setCurrent(double amps) {
        this.current.mut_setBaseUnitMagnitude(amps);
    }

    public void setTimestamp(double seconds) {
        this.timestamp.mut_setBaseUnitMagnitude(seconds);
    }
}
