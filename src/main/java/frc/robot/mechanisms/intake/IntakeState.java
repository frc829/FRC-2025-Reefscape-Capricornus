package frc.robot.mechanisms.intake;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class IntakeState implements Cloneable {

    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);


    public LinearVelocity getVelocity() {
        return velocity;
    }

    public IntakeState withVelocity(LinearVelocity velocity){
        this.velocity.mut_replace(velocity);
        return this;
    }

    public IntakeState withTimestamp(Time timestamp){
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public IntakeState withIntakeState(IntakeState intakeState){
        this.velocity.mut_replace(intakeState.velocity);
        this.timestamp.mut_replace(intakeState.timestamp);
        return this;
    }

    @Override
    public IntakeState clone() {
        try {
            IntakeState toReturn =  (IntakeState) super.clone();
            toReturn.velocity.mut_replace(velocity);
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
