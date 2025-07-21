package digilib.motorControllers;

public interface MotorController {

    double getVoltageVolts();

    double getCurrentAmps();

    double getPosition();

    double getVelocity();

    double getAcceleration();

    void applyVoltage(double voltageVolts);

    void applyCurrent(double currentAmps);

    void applyPositionUsingVoltage(double position);

    void applyPositionUsingCurrent(double position);

    void applyVelocityUsingVoltage(double velocity);

    void applyVelocityUsingCurrent(double velocity);

}
