package frc.robot.mechanisms.wrist;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface WristRequest {

    public void apply(WristControlParameters parameters, Wrist wrist);

    public class Hold implements WristRequest {
        @Override
        public void apply(WristControlParameters parameters, Wrist wrist) {

        }
    }

    public class Position implements WristRequest {
        private final MutAngle position = Radians.mutable(0.0);

        @Override
        public void apply(WristControlParameters parameters, Wrist wrist) {
            if(position.lte(parameters.getMaxAngle()) && position.gte(parameters.getMinAngle())) {
                wrist.setPosition(position);
            }else{
                wrist.setPosition(parameters.getCurrentState().getPosition());
            }
        }

        public Position withPosition(Angle position){
            this.position.mut_replace(position);
            return this;
        }
    }

    public class Velocity implements WristRequest {
        private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);

        @Override
        public void apply(WristControlParameters parameters, Wrist wrist) {
            if(parameters.getCurrentState().getPosition().lte(parameters.getMaxAngle()) && parameters.getCurrentState().getPosition().gte(parameters.getMinAngle())){
                wrist.setVelocity(velocity);
            }else{
                wrist.setVelocity(RadiansPerSecond.of(0.0));
            }
        }

        public Velocity withVelocity(AngularVelocity velocity){
            this.velocity.mut_replace(velocity);
            return this;
        }
    }

}
