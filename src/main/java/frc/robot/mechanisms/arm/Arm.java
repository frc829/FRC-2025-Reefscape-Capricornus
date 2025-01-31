package frc.robot.mechanisms.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public abstract class Arm {

    public enum ControlState {
        POSITION,
        VELOCITY,
        HOLD,
    }

    protected final ArmControlParameters armControlParameters;
    protected final ArmState lastArmState = new ArmState();
    protected final ArmState armState = new ArmState();
    private ArmRequest armRequest = new ArmRequest.Hold();

    public Arm(ArmControlParameters armControlParameters) {
        this.armControlParameters = armControlParameters;
    }

    public abstract void updateSimState(double dtSeconds);

    public void setControl(ArmRequest request){
        if(armRequest != request){
            armRequest = request;
        }
        armControlParameters.withArmState(armState);
        request.apply(armControlParameters, this);
    }

    public final ArmState getState(){
        return armState;
    }

    public final ArmState getStateCopy(){
        return armState.clone();
    }

    public ArmState getLastArmState() {
        return lastArmState;
    }

    public abstract void updateTelemetry();

    public abstract boolean setNeutralModeToBrake();

    public abstract boolean setNeutralModeToCoast();

    public abstract void setVelocity(AngularVelocity velocity);

    public abstract void setPosition(Angle position);

    public abstract void setHold();

    public abstract void resetPosition();

    public void update(){
        updateTelemetry();
    }
}
