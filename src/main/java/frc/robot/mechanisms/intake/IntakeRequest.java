package frc.robot.mechanisms.intake;

public interface IntakeRequest {

    public boolean apply(IntakeControlParameters parameters, Intake intake);

    public class Idle implements IntakeRequest {
        @Override
        public boolean apply(IntakeControlParameters parameters, Intake intake) {
            return false;
        }
    }


}
