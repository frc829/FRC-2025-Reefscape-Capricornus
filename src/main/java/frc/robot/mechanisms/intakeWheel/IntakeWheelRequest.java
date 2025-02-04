package frc.robot.mechanisms.intakeWheel;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;

public interface IntakeWheelRequest {

    public void apply(IntakeWheel intakeWheel);

    public class Velocity implements IntakeWheelRequest {
        private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);

        @Override
        public void apply(IntakeWheel intakeWheel) {
            intakeWheel.setVelocity(velocity);
        }

        public Velocity withVelocity(LinearVelocity velocity) {
            this.velocity.mut_replace(velocity);
            return this;
        }
    }


}
