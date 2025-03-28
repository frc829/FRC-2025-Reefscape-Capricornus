package digilib.elevator;

public interface Elevator {

    ElevatorState getState();

    void setPosition(double setpointMeters);

    void setVelocity(double setpointScalar);

    void setVoltage(double volts);

    void update();

    void updateSimState(double dt, double supplyVoltage);
}
