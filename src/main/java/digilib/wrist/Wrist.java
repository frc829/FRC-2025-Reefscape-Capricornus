package digilib.wrist;

public interface Wrist {

    WristState getState();

    void setPosition(double setpointRotations);

    void setVelocity(double setpointScalar);

    void update();

    void updateSimState(double dt, double supplyVoltage);
}
