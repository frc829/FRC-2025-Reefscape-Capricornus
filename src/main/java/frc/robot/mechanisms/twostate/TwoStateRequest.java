package frc.robot.mechanisms.twostate;

public interface TwoStateRequest {

    public boolean apply(TwoStateParameters parameters, TwoState twoState);

    public class Idle implements TwoStateRequest {
        @Override
        public boolean apply(TwoStateParameters parameters, TwoState twoState) {
            return true;
        }
    }

    public class SetToState0 implements TwoStateRequest {
        @Override
        public boolean apply(TwoStateParameters parameters, TwoState twoState) {
            if(parameters.getCurrentState() != TwoStateValue.STATE_0){
                return twoState.setToState0();
            }
            return true;
        }
    }

    public class SetToState1 implements TwoStateRequest {
        @Override
        public boolean apply(TwoStateParameters parameters, TwoState twoState) {
            if(parameters.getCurrentState() != TwoStateValue.STATE_1){
                return twoState.setToState1();
            }
            return true;
        }
    }


}
