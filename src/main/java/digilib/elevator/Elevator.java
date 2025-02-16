package digilib.elevator;

import digilib.MotorControllerType;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface Elevator {

    MotorControllerType getMotorControllerType();

    Distance getMaxPosition();

    Distance getMinPosition();

    LinearVelocity getMaxVelocity();

    ElevatorState getState();

    void setControl(ElevatorRequest request);

    void setPosition(Distance position);

    void setVelocity(LinearVelocity velocity);

    void setVoltage(Voltage voltage);

    void resetPosition();

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);
}
