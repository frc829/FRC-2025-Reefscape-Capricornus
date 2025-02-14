package digilib.wrist;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface WristRequest {

    void apply(Wrist wrist);

    class Position implements WristRequest {
        private final MutAngle position = Radians.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            if (wrist.isHoldEnabled()) {
                wrist.disableHold();
            }
            WristState wristState = wrist.getState();
            if (position.lte(wrist.getMaxAngle()) && position.gte(wrist.getMinAngle())) {
                wrist.setPosition(position);
            } else if(wristState.getPosition().gte(wrist.getMaxAngle()) && position.lte(wrist.getMaxAngle())){
                wrist.setPosition(position);
            } else if(wristState.getPosition().lte(wrist.getMinAngle()) && position.gte(wrist.getMinAngle())){
                wrist.setPosition(position);
            }else {
                wrist.setVelocity(RadiansPerSecond.of(0.0));
            }
        }

        public Position withPosition(double radians) {
            this.position.mut_setBaseUnitMagnitude(radians);
            return this;
        }
    }

    class Velocity implements WristRequest {
        private final MutDimensionless maxVelocityValue = Value.mutable(0.0);
        private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            if (wrist.isHoldEnabled()) {
                wrist.disableHold();
            }
            WristState state = wrist.getState();
            if (state.getPosition().lte(wrist.getMaxAngle()) && state.getPosition().gte(wrist.getMinAngle())) {
                velocity.mut_setMagnitude(maxVelocityValue.baseUnitMagnitude() * wrist.getMaxVelocity().baseUnitMagnitude());
            } else if(state.getPosition().gte(wrist.getMaxAngle()) && maxVelocityValue.baseUnitMagnitude() < 0.0){
                velocity.mut_setMagnitude(maxVelocityValue.baseUnitMagnitude() * wrist.getMaxVelocity().baseUnitMagnitude());
            } else if(state.getPosition().lte(wrist.getMinAngle()) && maxVelocityValue.baseUnitMagnitude() > 0.0){
                velocity.mut_setMagnitude(maxVelocityValue.baseUnitMagnitude() * wrist.getMaxVelocity().baseUnitMagnitude());
            }else {
                velocity.mut_setMagnitude(0.0);
            }
            wrist.setVelocity(velocity);
        }

        public Velocity withVelocity(double value) {
            this.maxVelocityValue.mut_setBaseUnitMagnitude(value);
            return this;
        }
    }

    class VoltageRequest implements WristRequest {
        private final MutVoltage voltage = Volts.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            if (wrist.isHoldEnabled()) {
                wrist.disableHold();
            }
            wrist.setVoltage(voltage);
        }

        public VoltageRequest withVoltage(Voltage voltage) {
            this.voltage.mut_replace(voltage);
            return this;
        }
    }

    class Hold implements WristRequest {
        private final MutAngle holdPosition = Radians.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            boolean isHoldEnabled = wrist.isHoldEnabled();
            if (!isHoldEnabled) {
                wrist.enableHold();
                holdPosition.mut_replace(wrist.getState().getPosition());
            }
            wrist.setPosition(holdPosition);
        }
    }
}
