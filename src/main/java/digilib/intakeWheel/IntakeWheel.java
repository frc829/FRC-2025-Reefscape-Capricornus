package digilib.intakeWheel;

import digilib.MotorControllerType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeWheel {

    MotorControllerType getMotorControllerType();

    AngularVelocity getMaxVelocity();

    IntakeWheelState getState();

    void setControl(IntakeWheelRequest request);

    void setVelocity(Dimensionless maxPercent);

    void setVoltage(Voltage voltage);

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dtSeconds, double supplyVoltage);
}
