package digilib.arm;

public interface Arm {

    ArmState getState();

    void setPosition(double setpointRotations);

    void setVelocity(double setpointScalar);

    void update();

    void updateSimState(double dt, double supplyVoltage);
}
