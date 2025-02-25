package digilib.wrist;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class WristState {

    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngle setpoint = Radians.mutable(0.0);
    private final MutAngle absolutePosition = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutAngularVelocity absoluteVelocity = RadiansPerSecond.mutable(0.0);
    private final MutVoltage voltage = Volts.mutable(0.0);
    private String status = "";

    public Angle getAngle() {
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

    public String getAbsoluteEncoderStatus() {
        return status;
    }

    public void setPosition(double radians) {
        this.position.mut_setBaseUnitMagnitude(radians);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint.mut_setBaseUnitMagnitude(setpoint);
    }

    public void setAbsolutePosition(double radians) {
        this.absolutePosition.mut_setBaseUnitMagnitude(radians);
    }

    public void setVelocity(double radiansPerSecond) {
        this.velocity.mut_setBaseUnitMagnitude(radiansPerSecond);
    }

    public void setAbsoluteVelocity(double radiansPerSecond) {
        this.absoluteVelocity.mut_setBaseUnitMagnitude(radiansPerSecond);
    }

    public void setVoltage(double voltage) {
        this.voltage.mut_setBaseUnitMagnitude(voltage);
    }

    public void setAbsoluteEncoderStatus(String status) {
        this.status = status;
    }
}
