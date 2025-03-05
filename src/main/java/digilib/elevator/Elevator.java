package digilib.elevator;

import digilib.MotorControllerType;

public interface Elevator {

    MotorControllerType getMotorControllerType();

    double getMinHeightMeters();

    double getMaxHeightMeters();

    double getMaxVelocityMPS();

    ElevatorState getState();

    void setPosition(double setpointMeters);

    void setVelocity(double setpointScalar);

    void setVoltage(double volts);

    void resetPosition();

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);
}
