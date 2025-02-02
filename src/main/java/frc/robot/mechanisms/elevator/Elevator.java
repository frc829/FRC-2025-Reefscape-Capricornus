package frc.robot.mechanisms.elevator;

import edu.wpi.first.units.measure.*;

public interface Elevator {

    public enum ControlState {
        POSITION,
        VELOCITY,
        HOLD,
    }

    public void updateSimState(Time dt, Voltage supplyVoltage);

    public void setControl(ElevatorRequest request);

    public ElevatorState getState();

    public ElevatorState getStateCopy();

    public ElevatorState getLastArmState();

    public void updateTelemetry();

    public boolean setNeutralModeToBrake();

    public boolean setNeutralModeToCoast();

    public void setVelocity(LinearVelocity velocity);

    public void setPosition(Distance position);

    public void setHold();

    public void resetPosition();

    public void update();

    public ElevatorRequest createHoldRequest();

    public ElevatorRequest createPositionRequest();

    public ElevatorRequest createVelocityRequest();
}
