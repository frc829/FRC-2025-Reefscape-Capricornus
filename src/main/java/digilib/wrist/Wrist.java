package digilib.wrist;

import digilib.MotorControllerType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface Wrist {

    MotorControllerType getMotorControllerType();

    Angle getMaxAngle();

    Angle getMinAngle();

    AngularVelocity getMaxVelocity();

    WristState getState();

    boolean isHoldEnabled();

    void setControl(WristRequest request);

    void setPosition(Angle position);

    void setVelocity(AngularVelocity velocity);

    void setVoltage(Voltage voltage);

    void resetPosition();

    void enableHold();

    void disableHold();

    void update();

    void updateState();

    void updateTelemetry();

    public void updateSimState(double dt, double supplyVoltage);
}
