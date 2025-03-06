package digilib.wrist;

import digilib.MotorControllerType;

public interface Wrist {

    MotorControllerType getMotorControllerType();

    double getMinAngleRotations();

    double getMaxAngleRotations();

    double getMaxVelocityRPS();

    WristState getState();

    void setPosition(double setpointRotations);

    void setVelocity(double setpointScalar);

    void setVoltage(double volts);

    void resetPosition();

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);
}
