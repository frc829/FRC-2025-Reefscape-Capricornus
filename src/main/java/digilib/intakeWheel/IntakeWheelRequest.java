package digilib.intakeWheel;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public interface IntakeWheelRequest {

    void apply(IntakeWheel intakeWheel);

    class Velocity implements IntakeWheelRequest {
        private final MutDimensionless maxVelocityValue = Value.mutable(0.0);
        private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);

        @Override
        public void apply(IntakeWheel intakeWheel) {
            velocity.mut_setBaseUnitMagnitude(maxVelocityValue.baseUnitMagnitude() * intakeWheel.getMaxVelocity().baseUnitMagnitude());
            intakeWheel.setVelocity(velocity);
        }

        public Velocity withVelocity(double value) {
            this.maxVelocityValue.mut_setMagnitude(value);
            return this;
        }
    }

    class Idle implements IntakeWheelRequest {
        @Override
        public void apply(IntakeWheel intakeWheel) {
            intakeWheel.setIdle();
        }
    }

    class VoltageRequest implements IntakeWheelRequest {
        private final MutVoltage voltage = Volts.mutable(0.0);

        @Override
        public void apply(IntakeWheel intakeWheel) {
            intakeWheel.setVoltage(voltage);
        }

        public VoltageRequest withVoltage(Voltage voltage) {
            this.voltage.mut_replace(voltage);
            return this;
        }
    }


}
