package frc.robot.mechanisms.twostate;

public abstract class TwoState {

    protected final TwoStateParameters twoStateParameters;
    protected final TwoStateValue lastState = TwoStateValue.STATE_0;
    protected final TwoStateValue currentState = TwoStateValue.STATE_0;
    protected final SimTwoState simTwoState = new SimTwoState();
    private TwoStateRequest twoStateRequest = new TwoStateRequest.Idle();

    public TwoState(TwoStateParameters twoStateParameters) {
        this.twoStateParameters = twoStateParameters;
    }

    public void updateSimState(TwoStateValue state) {
        simTwoState.update(state);
    }

    public void setControl(TwoStateRequest request){
        if(twoStateRequest != request){
            twoStateRequest = request;
        }
        twoStateParameters.withTwoFieldElementState(currentState);
        request.apply(twoStateParameters, this);
    }

    public final TwoStateValue getState(){
        return currentState;
    }

    public final TwoStateValue getStateCopy(){
        TwoStateValue toReturn = currentState;
        return toReturn;
    }

    public TwoStateValue getLastState() {
        return lastState;
    }

    public abstract void updateTelemetry();

    public abstract boolean setToState0();

    public abstract boolean setToState1();

    public void update(){
        updateTelemetry();
    }



}
