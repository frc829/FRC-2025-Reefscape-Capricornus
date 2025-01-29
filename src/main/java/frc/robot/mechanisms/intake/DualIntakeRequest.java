package frc.robot.mechanisms.intake;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;

public interface DualIntakeRequest {

    public void apply(DualIntakeControlParameters parameters, DualIntake dualIntake);

    public class Idle implements DualIntakeRequest {
        @Override
        public void apply(DualIntakeControlParameters parameters, DualIntake dualIntake) {
            dualIntake.setIdle();
        }
    }

    public class Velocity implements DualIntakeRequest {
        private final MutLinearVelocity intake0Velocity = MetersPerSecond.mutable(0.0);
        private final MutLinearVelocity intake1Velocity = MetersPerSecond.mutable(0.0);


        @Override
        public void apply(DualIntakeControlParameters parameters, DualIntake dualIntake) {
            dualIntake.setVelocity(intake0Velocity, intake1Velocity);
        }

        public Velocity withVelocity(LinearVelocity intakeVelocity0, LinearVelocity intakeVelocity1) {
            this.intake0Velocity.mut_replace(intakeVelocity0);
            this.intake1Velocity.mut_replace(intakeVelocity1);
            return this;
        }
    }


}
