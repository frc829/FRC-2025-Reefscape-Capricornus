package frc.robot.mechanisms.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public abstract class Elevator {

    public enum ControlState {
        POSITION,
        VELOCITY,
        HOLD,
    }

    protected final ElevatorControlParameters elevatorControlParameters;
    protected final ElevatorState lastElevatorState = new ElevatorState();
    protected final ElevatorState elevatorState = new ElevatorState();
    protected final SimElevator simElevator = new SimElevator();
    private ElevatorRequest elevatorRequest = new ElevatorRequest.Hold();

    public Elevator(ElevatorControlParameters elevatorControlParameters) {
        this.elevatorControlParameters = elevatorControlParameters;
    }

    public void updateSimState(double dtSeconds, double supplyVoltage) {
        simElevator.update(dtSeconds, supplyVoltage);
    }

    public void setControl(ElevatorRequest request) {
        if (elevatorRequest != request) {
            elevatorRequest = request;
        }
        elevatorControlParameters.withElevatorState(elevatorState);
        request.apply(elevatorControlParameters, this);
    }

    public final ElevatorState getState() {
        return elevatorState;
    }

    public final ElevatorState getStateCopy() {
        return elevatorState.clone();
    }

    public ElevatorState getLastElevatorState() {
        return lastElevatorState;
    }

    public abstract void updateTelemetry();

    public abstract boolean setNeutralModeToBrake();

    public abstract boolean setNeutralModeToCoast();

    public abstract void setVelocity(LinearVelocity velocity);

    public abstract void setPosition(Distance position);

    public abstract void setHold();

    public abstract void setFreeFall();

    public abstract void resetPosition();

    public void update() {
        updateTelemetry();
    }


}
