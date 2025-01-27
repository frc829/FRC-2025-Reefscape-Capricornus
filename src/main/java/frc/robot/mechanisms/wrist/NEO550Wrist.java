package frc.robot.mechanisms.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class NEO550Wrist extends Wrist {


    public NEO550Wrist(
            WristControlParameters wristControlParameters) {
        super(wristControlParameters);
    }

    @Override
    public boolean setNeutralModeToBrake() {
        return false;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        return false;
    }

    @Override
    public boolean setVelocity(AngularVelocity velocity) {
        return false;
    }

    @Override
    public boolean setPosition(Angle position) {
        return false;
    }

    @Override
    public boolean setHold() {
        return false;
    }

    @Override
    public boolean allowFall() {
        return false;
    }

    @Override
    public void resetPosition() {

    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public void updateTelemetry() {

    }
}
