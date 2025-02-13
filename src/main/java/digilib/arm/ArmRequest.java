package digilib.arm;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface ArmRequest {

    void apply(Arm arm);

    class Position implements ArmRequest {
        private final MutAngle position = Radians.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            if (arm.isHoldEnabled()) {
                arm.disableHold();
            }
            ArmState armState = arm.getState();
            if (position.lte(arm.getMaxAngle()) && position.gte(arm.getMinAngle())) {
                arm.setPosition(position);
            } else if (armState.getPosition().gte(arm.getMaxAngle()) && position.lte(arm.getMaxAngle())) {
                arm.setPosition(position);
            } else if (armState.getPosition().lte(arm.getMinAngle()) && position.gte(arm.getMinAngle())) {
                arm.setPosition(position);
            } else {
                arm.setVelocity(DegreesPerSecond.of(0.0));
            }
        }

        public Position withPosition(double radians) {
            this.position.mut_setBaseUnitMagnitude(radians);
            return this;
        }
    }

    class Velocity implements ArmRequest {
        private final MutDimensionless maxVelocityValue = Value.mutable(0.0);
        private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            if (arm.isHoldEnabled()) {
                arm.disableHold();
            }
            ArmState armState = arm.getState();
            if (armState.getPosition().lte(arm.getMaxAngle()) && armState.getPosition().gte(arm.getMinAngle())) {
                velocity.mut_setBaseUnitMagnitude(maxVelocityValue.baseUnitMagnitude() * arm.getMaxAngle().baseUnitMagnitude());
            } else if (armState.getPosition().gte(arm.getMaxAngle()) && maxVelocityValue.baseUnitMagnitude() < 0.0) {
                velocity.mut_setBaseUnitMagnitude(maxVelocityValue.baseUnitMagnitude() * arm.getMaxAngle().baseUnitMagnitude());
            } else if (armState.getPosition().lte(arm.getMinAngle()) && maxVelocityValue.baseUnitMagnitude() > 0.0) {
                velocity.mut_setBaseUnitMagnitude(maxVelocityValue.baseUnitMagnitude() * arm.getMaxAngle().baseUnitMagnitude());
            } else {
                velocity.mut_setBaseUnitMagnitude(0.0);
            }
            arm.setVelocity(velocity);
        }

        public Velocity withVelocity(double value) {
            this.maxVelocityValue.mut_setBaseUnitMagnitude(value);
            return this;
        }
    }

    class Hold implements ArmRequest {
        private final MutAngle holdPosition = Radians.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            boolean isHoldEnabled = arm.isHoldEnabled();
            if (!isHoldEnabled) {
                arm.enableHold();
                holdPosition.mut_replace(arm.getState().getPosition());
            }
            arm.setPosition(holdPosition);
        }
    }
}
