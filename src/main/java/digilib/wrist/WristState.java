package digilib.wrist;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class WristState {

    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngle absolutePosition = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutAngularVelocity absoluteVelocity = RadiansPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final MutVoltage voltage = Volts.mutable(0.0);
    private String status = "";

    public Angle getPosition() {
        return position;
    }

    public Angle getAbsolutePosition() {
        return absolutePosition;
    }

    public AngularVelocity getVelocity() {
        return velocity;
    }

    public AngularVelocity getAbsoluteVelocity() {
        return absoluteVelocity;
    }

    public Voltage getVoltage() {
        return voltage;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public String getAbsoluteEncoderStatus() {
        return status;
    }

    public WristState withPosition(double radians) {
        this.position.mut_setBaseUnitMagnitude(radians);
        return this;
    }

    public WristState withAbsolutePosition(double radians) {
        this.absolutePosition.mut_setBaseUnitMagnitude(radians);
        return this;
    }

    public WristState withVelocity(double radiansPerSecond) {
        this.velocity.mut_setBaseUnitMagnitude(radiansPerSecond);
        return this;
    }

    public WristState withAbsoluteVelocity(double radiansPerSecond) {
        this.absoluteVelocity.mut_setBaseUnitMagnitude(radiansPerSecond);
        return this;
    }

    public WristState withTimestamp(double seconds) {
        this.timestamp.mut_setBaseUnitMagnitude(seconds);
        return this;
    }

    public WristState withVoltage(double voltage) {
        this.voltage.mut_setBaseUnitMagnitude(voltage);
        return this;
    }

    public WristState withAbsoluteEncoderStatus(String status) {
        this.status = status;
        return this;
    }

}
