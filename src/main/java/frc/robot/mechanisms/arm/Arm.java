package frc.robot.mechanisms.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface Arm {

    enum ControlState {
        POSITION,
        VELOCITY,
        HOLD,
    }

    public void updateSimState(double dtSeconds);

    public void setControl(ArmRequest request);

    public ArmState getState();

    public ArmState getStateCopy();

    public ArmState getLastArmState();

    public void updateTelemetry();

    public boolean setNeutralModeToBrake();

    public boolean setNeutralModeToCoast();

    public void setVelocity(AngularVelocity velocity);

    public void setPosition(Angle position);

    public void setHold();

    public void resetPosition();

    public void update();

    public ArmRequest createHoldRequest();

    public ArmRequest createPositionRequest();

    public ArmRequest createVelocityRequest();
}
