package digilib.wrist;

import digilib.elevator.ElevatorState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public interface WristRequest {

    void apply(Wrist wrist);

    class Position implements WristRequest {
        private final MutAngle angle = Radians.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            WristState state = wrist.getState();
            if (state.getAngle().gte(wrist.getMaxAngle()) && angle.gt(wrist.getMaxAngle())) {
                wrist.setPosition(angle.mut_replace(wrist.getMaxAngle()));
            } else if (state.getAngle().lte(wrist.getMinAngle()) && angle.lt(wrist.getMinAngle())) {
                wrist.setPosition(angle.mut_replace(wrist.getMinAngle()));
            } else {
                wrist.setPosition(angle);
            }
            SmartDashboard.putNumber("Wrist Setpoint", angle.in(Degrees));
        }

        public WristRequest.Position withAngle(Angle angle) {
            this.angle.mut_replace(angle);
            return this;
        }
    }

    class Velocity implements WristRequest {
        private final MutDimensionless maxPercent = Value.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            WristState state = wrist.getState();
            if (state.getAngle().gte(wrist.getMaxAngle()) && maxPercent.gt(Value.of(0.0))) {
                wrist.setPosition(wrist.getMaxAngle());
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            } else if (state.getAngle().lte(wrist.getMinAngle()) && maxPercent.lt(Value.of(0.0))) {
                wrist.setPosition(wrist.getMinAngle());
                maxPercent.mut_setBaseUnitMagnitude(0.0);
            } else{
                wrist.setVelocity(maxPercent);
            }
        }

        public WristRequest.Velocity withVelocity(Dimensionless maxPercent) {
            this.maxPercent.mut_replace(maxPercent);
            return this;
        }
    }

    class VoltageRequest implements WristRequest {
        private final MutVoltage voltage = Volts.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            wrist.setVoltage(voltage);
        }

        public VoltageRequest withVoltage(Voltage voltage) {
            this.voltage.mut_replace(voltage);
            return this;
        }
    }
}
