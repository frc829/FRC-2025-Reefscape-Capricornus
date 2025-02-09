package digilib.wrist;

import digilib.arm.Arm;
import digilib.arm.ArmRequest;
import digilib.arm.ArmState;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface WristRequest {

    public void apply(Wrist wrist);

    public class Hold implements WristRequest {
        private final MutAngle holdPosition = Degrees.mutable(0.0);

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

    public class Position implements WristRequest {
        private final MutAngle position = Degrees.mutable(0.0);

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
                wrist.setVelocity(DegreesPerSecond.of(0.0));
            }
        }

        public Position withPosition(Angle position) {
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements WristRequest {
        private final MutDimensionless maxVelocityPercent = Percent.mutable(0.0);
        private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            if (wrist.isHoldEnabled()) {
                wrist.disableHold();
            }
            WristState state = wrist.getState();
            if (state.getPosition().lte(wrist.getMaxAngle()) && state.getPosition().gte(wrist.getMinAngle())) {
                velocity.mut_setMagnitude(maxVelocityPercent.baseUnitMagnitude() * wrist.getMaxVelocity().baseUnitMagnitude());
            } else if(state.getPosition().gte(wrist.getMaxAngle()) && maxVelocityPercent.baseUnitMagnitude() < 0.0){
                velocity.mut_setMagnitude(maxVelocityPercent.baseUnitMagnitude() * wrist.getMaxVelocity().baseUnitMagnitude());
            } else if(state.getPosition().lte(wrist.getMinAngle()) && maxVelocityPercent.baseUnitMagnitude() > 0.0){
                velocity.mut_setMagnitude(maxVelocityPercent.baseUnitMagnitude() * wrist.getMaxVelocity().baseUnitMagnitude());
            }else {
                velocity.mut_setMagnitude(0.0);
            }
            wrist.setVelocity(velocity);
        }

        public Velocity withVelocity(Dimensionless maxVelocityPercent) {
            this.maxVelocityPercent.mut_replace(maxVelocityPercent);
            return this;
        }
    }

}
