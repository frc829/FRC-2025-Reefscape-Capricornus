package digilib.intakeWheel;

import edu.wpi.first.units.measure.LinearVelocity;

public interface IntakeWheel {

    public void updateSimState(double dtSeconds, double supplyVoltage);

    public void setControl(IntakeWheelRequest request);

    public IntakeWheelState getState();

    public IntakeWheelState getStateCopy();

    public IntakeWheelState getLastIntakeState();

    public void updateTelemetry();

    public boolean setNeutralModeToBrake();

    public boolean setNeutralModeToCoast();

    public void setVelocity(LinearVelocity velocity);

    public void update();

    public LinearVelocity getMaxVelocity();
}
