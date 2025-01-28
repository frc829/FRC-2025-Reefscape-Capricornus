package frc.robot.mechanisms.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public abstract class Wrist {

    protected final WristControlParameters wristControlParameters;
    protected final WristState lastWristState = new WristState();
    protected final WristState wristState = new WristState();
    protected final SimWrist simWrist = new SimWrist();
    private WristRequest wristRequest = new WristRequest.Hold();

    public Wrist(WristControlParameters wristControlParameters) {
        this.wristControlParameters = wristControlParameters;
    }

    public void updateSimState(double dtSeconds, double supplyVoltage){
        simWrist.update(dtSeconds, supplyVoltage);
    }

    public void setControl(WristRequest request){
        if(wristRequest != request){
            wristRequest = request;
        }
        wristControlParameters.withWristState(wristState);
        request.apply(wristControlParameters, this);
    }

    public final WristState getState(){
        return wristState;
    }

    public final WristState getStateCopy(){
        return wristState.clone();
    }

    public WristState getLastWristState() {
        return lastWristState;
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
