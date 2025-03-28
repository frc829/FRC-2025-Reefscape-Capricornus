package digilib.intakeWheel;

public interface IntakeWheel {

    IntakeWheelState getState();

    void setVelocity(double setpointScalar);

    void setVoltage(double volts);

    void update();

    void updateSimState(double dtSeconds, double supplyVoltage);
}
