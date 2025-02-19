package digilib.wrist;

import digilib.MotorControllerType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

public interface Wrist {

    MotorControllerType getMotorControllerType();

    Angle getMaxAngle();

    Angle getMinAngle();

    AngularVelocity getMaxVelocity();

    WristState getState();

    void setControl(WristRequest request);

    void setPosition(Angle position);

    void setVelocity(Dimensionless maxPercent);

    void setVoltage(Voltage voltage);

    void resetPosition();

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);
}
