package digilib.intakeWheel;

import digilib.elevator.Elevator;
import digilib.elevator.ElevatorRequest;
import digilib.elevator.ElevatorState;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.units.measure.MutLinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

public interface IntakeWheelRequest {

    public void apply(IntakeWheel intakeWheel);

    public class Velocity implements IntakeWheelRequest {
        private final MutDimensionless maxVelocityPercent = Percent.mutable(0.0);
        private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);

        @Override
        public void apply(IntakeWheel intakeWheel) {
            velocity.mut_setMagnitude(maxVelocityPercent.baseUnitMagnitude() * intakeWheel.getMaxVelocity().baseUnitMagnitude());
            intakeWheel.setVelocity(velocity);

        }

        public Velocity withVelocity(Dimensionless maxVelocityPercent) {
            this.maxVelocityPercent.mut_replace(maxVelocityPercent);
            return this;
        }
    }


}
