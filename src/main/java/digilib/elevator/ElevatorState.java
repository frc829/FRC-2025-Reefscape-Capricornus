package digilib.elevator;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class ElevatorState {

    private final MutDistance position = Meters.mutable(0.0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final MutVoltage voltage = Volts.mutable(0.0);

    public Distance getPosition() {
        return position;
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

    public ElevatorState withPosition(double meters){
        this.position.mut_setBaseUnitMagnitude(meters);
        return this;
    }

    public ElevatorState withVelocity(double metersPerSecond){
        this.velocity.mut_setBaseUnitMagnitude(metersPerSecond);
        return this;
    }

    public ElevatorState withVoltage(double voltage){
        this.voltage.mut_setBaseUnitMagnitude(voltage);
        return this;
    }

    public ElevatorState withTimestamp(double seconds){
        this.timestamp.mut_setBaseUnitMagnitude(seconds);
        return this;
    }
}
