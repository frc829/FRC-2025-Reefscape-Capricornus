package frc.robot.mechanisms.wrist;

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
            if (position.lte(wrist.getMaxAngle()) && position.gte(wrist.getMinAngle())) {
                wrist.setPosition(position);
            } else {
                wrist.setVelocity(DegreesPerSecond.of(0.0));
            }
        }

        public Position withPosition(Angle position) {
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements WristRequest {
        private final MutAngularVelocity velocity = DegreesPerSecond.mutable(0.0);

        @Override
        public void apply(Wrist wrist) {
            if (wrist.isHoldEnabled()) {
                wrist.disableHold();
            }
            WristState wristState = wrist.getState();
            if (wristState.getPosition().lte(wrist.getMaxAngle()) && wristState.getPosition().gte(wrist.getMinAngle())) {
                wrist.setVelocity(velocity);
            } else {
                wrist.setVelocity(DegreesPerSecond.of(0.0));
            }
        }

        public Velocity withVelocity(AngularVelocity velocity) {
            this.velocity.mut_replace(velocity);
            return this;
        }
    }

}
