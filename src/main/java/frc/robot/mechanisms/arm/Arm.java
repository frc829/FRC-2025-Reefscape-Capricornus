package frc.robot.mechanisms.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public abstract class Arm {

    protected final ArmControlParameters armControlParameters;
    protected final ArmState lastArmState = new ArmState();
    protected final ArmState armState = new ArmState();
    protected final SimArm simArm = new SimArm();
    private ArmRequest armRequest = new ArmRequest.Hold();

    public Arm(ArmControlParameters armControlParameters) {
        this.armControlParameters = armControlParameters;
    }

    public void updateSimState(double dtSeconds, double supplyVoltage){
        simArm.update(dtSeconds, supplyVoltage);
    }

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

    public abstract boolean setVelocity(AngularVelocity velocity);

    public abstract boolean setPosition(Angle position);

    public abstract boolean setHold();

    public abstract boolean allowFall();

    public abstract void resetPosition();

    public void update(){
        updateTelemetry();
    }



}
