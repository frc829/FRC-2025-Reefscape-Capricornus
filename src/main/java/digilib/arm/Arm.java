package digilib.arm;

import digilib.MotorControllerType;

public interface Arm {

    MotorControllerType getMotorControllerType();

    double getMinAngleRotations();

    double getMaxAngleRotations();

    double getMaxVelocityRPS();

    ArmState getState();

    void setControl(ArmRequest request);

    void setPosition(double setpointRotations);

    void setVelocity(double setpointScalar);

    void setVoltage(double volts);

    void resetPosition();

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);
}
