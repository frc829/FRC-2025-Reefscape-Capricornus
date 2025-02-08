package digilib.intakeWheel;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
