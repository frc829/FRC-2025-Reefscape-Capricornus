package digilib.arm;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class ArmState {

    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutAngle absolutePosition = Radians.mutable(0.0);
    private final MutAngularVelocity absoluteVelocity = RadiansPerSecond.mutable(0.0);
    private final MutVoltage voltage = Volts.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
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

    public ArmState withPosition(double radians) {
        this.position.mut_setBaseUnitMagnitude(radians);
        return this;
    }

    public ArmState withAbsolutePosition(double radians) {
        this.absolutePosition.mut_setBaseUnitMagnitude(radians);
        return this;
    }

    public ArmState withVelocity(double radians) {
        this.velocity.mut_setBaseUnitMagnitude(radians);
        return this;
    }

    public ArmState withAbsoluteVelocity(double radiansPerSecond) {
        this.absoluteVelocity.mut_setBaseUnitMagnitude(radiansPerSecond);
        return this;
    }

    public ArmState withVoltage(double volts) {
        this.voltage.mut_setBaseUnitMagnitude(volts);
        return this;
    }

    public ArmState withTimestamp(double seconds) {
        this.timestamp.mut_setBaseUnitMagnitude(seconds);
        return this;
    }

    public ArmState withAbsoluteEncoderStatus(String status) {
        this.status = status;
        return this;
    }
}
