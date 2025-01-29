package frc.robot.mechanisms.intake;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class DualIntakeState implements Cloneable {

    private final MutLinearVelocity intake0Velocity = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity intake1Velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);

    public LinearVelocity getIntake0Velocity() {
        return intake0Velocity;
    }

    public MutLinearVelocity getIntake1Velocity() {
        return intake1Velocity;
    }

    public DualIntakeState withVelocity(LinearVelocity intake0Velocity, LinearVelocity intake1Velocity) {
        this.intake0Velocity.mut_replace(intake0Velocity);
        this.intake1Velocity.mut_replace(intake1Velocity);
        return this;
    }

    public DualIntakeState withTimestamp(Time timestamp){
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public DualIntakeState withIntakeState(DualIntakeState dualIntakeState){
        this.intake0Velocity.mut_replace(dualIntakeState.intake0Velocity);
        this.intake1Velocity.mut_replace(dualIntakeState.intake1Velocity);
        this.timestamp.mut_replace(dualIntakeState.timestamp);
        return this;
    }

    @Override
    public DualIntakeState clone() {
        try {
            DualIntakeState toReturn =  (DualIntakeState) super.clone();
            toReturn.intake0Velocity.mut_replace(intake0Velocity);
            toReturn.intake1Velocity.mut_replace(intake1Velocity);
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }


}
