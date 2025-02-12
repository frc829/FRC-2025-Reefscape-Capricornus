package digilib.intakeWheel;

import digilib.MotorControllerType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeWheel {

    public MotorControllerType getMotorControllerType();

    public void updateSimState(double dtSeconds, double supplyVoltage);

    public void setControl(IntakeWheelRequest request);

    public IntakeWheelState getState();

    public IntakeWheelState getStateCopy();

    public IntakeWheelState getLastIntakeState();

    public void updateTelemetry();

    public boolean setNeutralModeToBrake();

    public boolean setNeutralModeToCoast();

    public void setVelocity(AngularVelocity velocity);

    public void setIdle();

    public void setVoltage(Voltage voltage);

    public void update();

    public AngularVelocity getMaxVelocity();
}
