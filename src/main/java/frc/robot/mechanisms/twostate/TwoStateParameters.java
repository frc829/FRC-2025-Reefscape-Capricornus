package frc.robot.mechanisms.twostate;

public class TwoStateParameters {

    private TwoStateValue currentState = TwoStateValue.STATE_0;

    public TwoStateValue getCurrentState() {
        return currentState;
    }

    public TwoStateParameters withTwoFieldElementState(TwoStateValue state) {
        currentState = state;
        return this;
    }


}
