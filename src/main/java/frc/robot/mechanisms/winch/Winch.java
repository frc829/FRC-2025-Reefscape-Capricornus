package frc.robot.mechanisms.winch;

import edu.wpi.first.units.measure.Dimensionless;

import static frc.robot.mechanisms.winch.WinchRequest.Idle;

public abstract class Winch {

    public enum ControlState {
        DUTY_CYCLE,
        IDLE
    }

    protected final WinchState lastWinchState = new WinchState();
    protected final WinchState winchState = new WinchState();
    protected final SimWinch simArm = new SimWinch();
    private WinchRequest winchRequest = new Idle();

    public Winch() {
    }

    public void updateSimState(double dtSeconds, double supplyVoltage){
        simArm.update(dtSeconds, supplyVoltage);
    }

    public void setControl(WinchRequest request){
        if(winchRequest != request){
            winchRequest = request;
        }
        request.apply(this);
    }

    public final WinchState getState(){
        return winchState;
    }

    public final WinchState getStateCopy(){
        return winchState.clone();
    }

    public WinchState getLastArmState() {
        return lastWinchState;
    }

    public abstract void updateTelemetry();

    public abstract boolean setNeutralModeToBrake();

    public abstract boolean setNeutralModeToCoast();

    public abstract void setDutyCycle(Dimensionless dutyCycle);

    public abstract void setIdle();

    public void update(){
        updateTelemetry();
    }
}
