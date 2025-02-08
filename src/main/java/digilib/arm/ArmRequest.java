package digilib.arm;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.*;

import static edu.wpi.first.units.Units.*;

public interface ArmRequest {

    public void apply(Arm arm);

    public class Hold implements ArmRequest {
        private final MutAngle holdPosition = Radians.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            boolean isHoldEnabled = arm.isHoldEnabled();
            if (!isHoldEnabled) {
                arm.enableHold();
                holdPosition.mut_replace(arm.getState().getPosition());
            }
            SmartDashboard.putNumber("Hold Position [deg]", holdPosition.in(Degrees));
            arm.setPosition(holdPosition);
        }
    }

    public class Position implements ArmRequest {
        private final MutAngle position = Radians.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            if (arm.isHoldEnabled()) {
                arm.disableHold();
            }
            if (position.lte(arm.getMaxAngle()) && position.gte(arm.getMinAngle())) {
                arm.setPosition(position);
            } else {
                arm.setVelocity(RadiansPerSecond.of(0.0));
            }
            SmartDashboard.putNumber("Position Setpoint [deg]", position.in(Degrees));
        }

        public Position withPosition(Angle position) {
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements ArmRequest {
        private final MutDimensionless maxVelocityPercent = Percent.mutable(0.0);
        private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);

        @Override
        public void apply(Arm arm) {
            if (arm.isHoldEnabled()) {
                arm.disableHold();
            }
            ArmState armState = arm.getState();
            if (armState.getPosition().lte(arm.getMaxAngle()) && armState.getPosition().gte(arm.getMinAngle())) {
                velocity.mut_setMagnitude(maxVelocityPercent.baseUnitMagnitude() * arm.getMaxVelocity().baseUnitMagnitude());
            } else {
                velocity.mut_setMagnitude(0.0);
            }
            arm.setVelocity(velocity);
        }

        public Velocity withVelocity(Dimensionless maxVelocityPercent) {
            this.maxVelocityPercent.mut_replace(maxVelocityPercent);
            return this;
        }
    }
}
