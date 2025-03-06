package digilib.intakeWheel;

import digilib.MotorControllerType;

public interface IntakeWheel {

    MotorControllerType getMotorControllerType();

    double getMaxVelocityRPS();

    IntakeWheelState getState();

    void setVelocity(double setpointScalar);

    void setVoltage(double volts);

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dtSeconds, double supplyVoltage);
}
