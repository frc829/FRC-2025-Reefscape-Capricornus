package digilib.elevator;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class ElevatorState {

    private final MutDistance height = Meters.mutable(0.0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final MutVoltage voltage = Volts.mutable(0.0);

    public Distance getHeight() {
        return height;
    }

    public LinearVelocity getVelocity() {
        return velocity;
    }

    public Voltage getVoltage() {
        return voltage;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public void setHeight(double heightMeters) {
        this.height.mut_setBaseUnitMagnitude(heightMeters);
    }

    public void setHeight(Distance height) {
        this.height.mut_replace(height);
    }

    public void setVelocity(double velocityMetersPerSecond) {
        this.velocity.mut_setBaseUnitMagnitude(velocityMetersPerSecond);
    }

    public void setVelocity(LinearVelocity velocity) {
        this.velocity.mut_replace(velocity);
    }

    public void setVoltage(double volts) {
        this.voltage.mut_setBaseUnitMagnitude(volts);
    }

    public void setVoltage(Voltage voltage) {
        this.voltage.mut_replace(voltage);
    }

    public void setTimestamp(double seconds) {
        this.timestamp.mut_setBaseUnitMagnitude(seconds);
    }
}
