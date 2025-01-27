package frc.robot.mechanisms.twostate;

import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidTwoState extends TwoState {

    private final Solenoid solenoid;

    public SolenoidTwoState(
            TwoStateParameters parameters,
            Solenoid solenoid) {
        super(parameters);
        this.solenoid = solenoid;
    }

    @Override
    public void updateTelemetry() {

    }

    @Override
    public boolean setToState0() {
        solenoid.set(false);
        return true;
    }

    @Override
    public boolean setToState1() {
        solenoid.set(true);
        return true;
    }

    @Override
    public void update() {
        super.update();
    }
}
