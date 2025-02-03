package frc.robot.mechanisms.winch;

import edu.wpi.first.units.measure.Dimensionless;

public interface Winch {

    public void updateSimState(double dt, double supplyVoltage);

    public void setControl(WinchRequest request);

    public WinchState getState();

    public WinchState getStateCopy();

    public WinchState getLastArmState();

    public void enableHold();

    public void disableHold();

    public boolean isHoldEnabled();

    public void updateTelemetry();

    public boolean setNeutralModeToBrake();

    public boolean setNeutralModeToCoast();

    public void setDutyCycle(Dimensionless dutyCycle);

    public void update();
}
