package digilib.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface Elevator {

    public void updateSimState(double dt, double supplyVoltage);

    public void setControl(ElevatorRequest request);

    public ElevatorState getState();

    public ElevatorState getStateCopy();

    public ElevatorState getLastArmState();

    public void enableHold();

    public void disableHold();

    public boolean isHoldEnabled();

    public void updateTelemetry();

    public boolean setNeutralModeToBrake();

    public boolean setNeutralModeToCoast();

    public void setVelocity(LinearVelocity velocity);

    public void setPosition(Distance position);

    public void setVoltage(Voltage voltage);

    public void resetPosition();

    public void update();

    public Distance getMaxPosition();

    public Distance getMinPosition();

    public LinearVelocity getMaxVelocity();

}
