package digilib.intakeWheel;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface IntakeWheelRequest {

    void apply(IntakeWheel intakeWheel);

    class Velocity implements IntakeWheelRequest {
        private final MutDimensionless maxPercent = Value.mutable(0.0);

        @Override
        public void apply(IntakeWheel intakeWheel) {
            intakeWheel.setVelocity(maxPercent);
        }

        public IntakeWheelRequest.Velocity withVelocity(Dimensionless maxPercent) {
            this.maxPercent.mut_replace(maxPercent);
            return this;
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
