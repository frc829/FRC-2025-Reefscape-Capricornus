package digilib.motorControllers;

public interface MotorController {

    double getVoltage();

    double getCurrentAmps();

    double getPosition();

    double getVelocity();

    double getAcceleration();

    void applyVoltage(double voltage);

    void applyCurrentAmps(double currentAmps);

    void applyPositionWithVoltage(double position);

    void applyPositionWithCurrent(double position);

    void applyVelocityWithVoltage(double velocity);

    void applyVelocityWithCurrent(double velocity);
}
