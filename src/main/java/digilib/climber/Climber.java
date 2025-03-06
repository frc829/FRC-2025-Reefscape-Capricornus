package digilib.climber;

import digilib.MotorControllerType;

public interface Climber {

    MotorControllerType getMotorControllerType();

    double getMinLengthMeters();

    double getMaxLengthMeters();

    double getMaxVelocityMPS();

    ClimberState getState();

    void setVoltage(double volts);

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);
}
