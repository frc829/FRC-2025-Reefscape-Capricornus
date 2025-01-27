package frc.robot.mechanisms.wrist;

public interface WristRequest {

    public boolean apply(WristControlParameters parameters, Wrist wrist);

    public class Hold implements WristRequest {
        @Override
        public boolean apply(WristControlParameters parameters, Wrist wrist) {
            return false;
        }
    }

    public class FreeFall implements WristRequest {
        @Override
        public boolean apply(WristControlParameters parameters, Wrist wrist) {
            return false;
        }
    }


}
