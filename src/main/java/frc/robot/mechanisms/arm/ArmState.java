package frc.robot.mechanisms.arm;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class ArmState implements Cloneable {

    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);

    public Angle getPosition() {
        return position;
    }

    public AngularVelocity getVelocity() {
        return velocity;
    }

    public ArmState withPosition(Angle position){
        this.position.mut_replace(position);
        return this;
    }

    public ArmState withVelocity(AngularVelocity velocity){
        this.velocity.mut_replace(velocity);
        return this;
    }

    public ArmState withTimestamp(Time timestamp){
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public ArmState withArmState(ArmState armState){
        this.position.mut_replace(armState.position);
        this.velocity.mut_replace(armState.velocity);
        this.timestamp.mut_replace(armState.timestamp);
        return this;
    }

    @Override
    public ArmState clone() {
        try {
            ArmState toReturn =  (ArmState) super.clone();
            toReturn.position.mut_replace(position);
            toReturn.velocity.mut_replace(velocity);
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
