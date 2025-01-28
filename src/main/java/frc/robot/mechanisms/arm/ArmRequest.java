package frc.robot.mechanisms.arm;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface ArmRequest {

    public void apply(ArmControlParameters parameters, Arm arm);

    public class Hold implements ArmRequest {
        @Override
        public void apply(ArmControlParameters parameters, Arm arm) {

        }
    }

    public class Position implements ArmRequest {
        private final MutAngle position = Radians.mutable(0.0);

        @Override
        public void apply(ArmControlParameters parameters, Arm arm) {
            if(position.lte(parameters.getMaxAngle()) && position.gte(parameters.getMinAngle())) {
                arm.setPosition(position);
            }else{
                arm.setPosition(parameters.getCurrentState().getPosition());
            }
        }

        public Position withPosition(Angle position){
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements ArmRequest {
        private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);

        @Override
        public void apply(ArmControlParameters parameters, Arm arm) {
            if(parameters.getCurrentState().getPosition().lte(parameters.getMaxAngle()) && parameters.getCurrentState().getPosition().gte(parameters.getMinAngle())){
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
