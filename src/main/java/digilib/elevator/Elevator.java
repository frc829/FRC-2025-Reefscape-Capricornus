package digilib.elevator;

import digilib.MotorControllerType;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface Elevator {

    MotorControllerType getMotorControllerType();

    Distance getMaxHeight();

    Distance getMinHeight();

    LinearVelocity getMaxVelocity();

    ElevatorState getState();

    void setControl(ElevatorRequest request);

    void setHeight(Distance height);

    void setVelocity(Dimensionless maxPercent);

    void setVoltage(Voltage voltage);

    void resetPosition();

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);
}
