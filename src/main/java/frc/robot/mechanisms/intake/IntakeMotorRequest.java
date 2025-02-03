package frc.robot.mechanisms.intake;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

import static edu.wpi.first.units.Units.MetersPerSecond;

public interface IntakeMotorRequest {

    public void apply(Intake intake);

    public class Idle implements IntakeMotorRequest {
        @Override
        public void apply(Intake intake) {

        }
    }

    public class Velocity implements IntakeMotorRequest {
        private final List<MutLinearVelocity> velocities;

        public Velocity(int numIntakeMotors) {
            velocities = IntStream.range(0, numIntakeMotors)
                    .mapToObj(motor -> MetersPerSecond.mutable(0.0))
                    .toList();
        }

        @Override
        public void apply(Intake intake) {

        }

        public Velocity withVelocity(LinearVelocity... velocities) {
            if (this.velocities.size() ==  velocities.length){

            }
            return this;
        }
    }


}
