package digilib.arm;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class ArmState {

    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutAngle absolutePosition = Radians.mutable(0.0);
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

    public void setPosition(Angle angle) {
        this.position.mut_replace(angle);
    }

    public void setAbsolutePosition(Angle angle) {
        this.absolutePosition.mut_replace(angle);
    }

    public void setVelocity(AngularVelocity angularVelocity) {
        this.velocity.mut_replace(angularVelocity);
    }

    public void setAbsoluteVelocity(AngularVelocity angularVelocity) {
        this.absoluteVelocity.mut_replace(angularVelocity);
    }

    public void setVoltage(Voltage volts) {
        this.voltage.mut_replace(volts);
    }

    public void setAbsoluteEncoderStatus(String status) {
        this.status = status;
    }
}
