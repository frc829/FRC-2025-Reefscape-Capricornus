package frc.robot.mechanisms.arm;

public interface ArmRequest {

    public boolean apply(ArmControlParameters parameters, Arm arm);

    public class Hold implements ArmRequest {
        @Override
        public boolean apply(ArmControlParameters parameters, Arm arm) {
            return false;
        }
    }

    public class FreeFall implements ArmRequest {
        @Override
        public boolean apply(ArmControlParameters parameters, Arm arm) {
            return false;
        }
    }


}
