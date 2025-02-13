package digilib.arm;

import digilib.MotorControllerType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface Arm {

    MotorControllerType getMotorControllerType();

    Angle getMaxAngle();

    Angle getMinAngle();

    AngularVelocity getMaxVelocity();

    ArmState getState();

    boolean isHoldEnabled();

    void setControl(ArmRequest request);

    void setPosition(Angle position);

    void setVelocity(AngularVelocity velocity);

    void setVoltage(Voltage voltage);

    void enableHold();

    void disableHold();

    void resetPosition();

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);
}
