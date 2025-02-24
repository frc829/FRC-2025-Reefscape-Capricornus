package digilib.arm;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public interface ArmRequest {

    void apply(Arm arm);

    class Position implements ArmRequest {
        private final MutAngle angle = Radians.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            ArmState state = arm.getState();
            if (state.getAngle().gte(arm.getMaxAngle()) && angle.gt(arm.getMaxAngle())) {
                arm.setPosition(angle.mut_replace(arm.getMaxAngle()));
            } else if (state.getAngle().lte(arm.getMinAngle()) && angle.lt(arm.getMinAngle())) {
                arm.setPosition(angle.mut_replace(arm.getMinAngle()));
            } else {
                arm.setPosition(angle);
            }
            SmartDashboard.putNumber("Arm Setpoint", angle.in(Degrees));
        }

        public Position withPosition(Angle angle) {
            this.angle.mut_replace(angle);
            return this;
        }
    }

    class Velocity implements ArmRequest {
        private final MutDimensionless maxPercent = Value.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            ArmState state = arm.getState();
            if (state.getAngle().gte(arm.getMaxAngle()) && maxPercent.gt(Value.of(0.0))) {
                arm.setPosition(arm.getMaxAngle());
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            } else if (state.getAngle().lte(arm.getMinAngle()) && maxPercent.lt(Value.of(0.0))) {
                arm.setPosition(arm.getMinAngle());
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            } else {
                arm.setVelocity(maxPercent);
            }
        }

        public Velocity withVelocity(Dimensionless maxPercent) {
            this.maxPercent.mut_replace(maxPercent);
            return this;
        }
    }

    class VoltageRequest implements ArmRequest {
        private final MutVoltage voltage = Volts.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            arm.setVoltage(voltage);
        }

        public VoltageRequest withVoltage(Voltage voltage) {
            this.voltage.mut_replace(voltage);
            return this;
        }
    }
}
