package frc.robot.mechanisms.arm;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface ArmRequest {

    public void apply(Arm arm);

    public class Hold implements ArmRequest {
        @Override
        public void apply(Arm arm) {
            arm.setHold();
        }
    }

    public class Position implements ArmRequest {
        private final MutAngle position = Radians.mutable(0.0);
        private final Angle minAngle;
        private final Angle maxAngle;

        public Position(Angle minAngle, Angle maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
        }

        @Override
        public void apply(Arm arm) {
            if(position.lte(maxAngle) && position.gte(minAngle)) {
                arm.setPosition(position);
            }else{
                arm.setPosition(position);
            }
        }

        public Position withPosition(Angle position){
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements ArmRequest {
        private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
        private final Angle minAngle;
        private final Angle maxAngle;

        public Velocity(Angle minAngle, Angle maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
        }
        @Override
        public void apply(Arm arm) {
            ArmState armState = arm.getState();
            if(armState.getPosition().lte(maxAngle) && armState.getPosition().gte(minAngle)){
                arm.setVelocity(velocity);
            }else{
                arm.setVelocity(RadiansPerSecond.of(0.0));
            }
        }

        public Velocity withVelocity(AngularVelocity velocity){
            this.velocity.mut_replace(velocity);
            return this;
        }
    }


}
