package frc.robot.mechanisms.intake;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class IntakeMotorState  implements Cloneable {

    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);


    public IntakeMotorState withVelocity(LinearVelocity velocity) {
        this.velocity.mut_replace(velocity);
        return this;
    }

    public IntakeMotorState withTimestamp(Time timestamp){
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public IntakeMotorState withIntakeState(IntakeMotorState intakeMotorState){
        this.velocity.mut_replace(intakeMotorState.velocity);
        this.timestamp.mut_replace(intakeMotorState.timestamp);
        return this;
    }

    @Override
    public IntakeMotorState clone() {
        try {
            IntakeMotorState toReturn =  (IntakeMotorState) super.clone();
            toReturn.velocity.mut_replace(velocity);
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }



}
