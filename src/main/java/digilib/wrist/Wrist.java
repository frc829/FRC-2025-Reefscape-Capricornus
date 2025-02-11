package digilib.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface Wrist {

    public void updateSimState(double dt, double supplyVoltage);

    public void setControl(WristRequest request);

    public WristState getState();

    public WristState getStateCopy();

    public WristState getLastArmState();

    public void enableHold();

    public void disableHold();

    public boolean isHoldEnabled();

    public void updateTelemetry();

    public boolean setNeutralModeToBrake();

    public boolean setNeutralModeToCoast();

    public void setVelocity(AngularVelocity velocity);

    public void setPosition(Angle position);

    public void setVoltage(Voltage voltage);

    public void resetPosition();

    public void update();

    public Angle getMaxAngle();

    public Angle getMinAngle();

    public AngularVelocity getMaxVelocity();



}
