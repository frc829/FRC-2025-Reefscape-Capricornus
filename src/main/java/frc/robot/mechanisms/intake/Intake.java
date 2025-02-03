package frc.robot.mechanisms.intake;

import edu.wpi.first.units.measure.LinearVelocity;

import java.util.List;

public interface Intake {

    public void updateSimState(double dtSeconds, double supplyVoltage);

    public void setControl(IntakeMotorRequest... request);

    public List<IntakeMotorState> getState();

    public List<IntakeMotorState> getStateCopy();

    public List<IntakeMotorState> getLastIntakeState();

    public boolean hasElement();

    public void updateTelemetry();

    public boolean setNeutralModeToBrake();

    public boolean setNeutralModeToCoast();

    public void setVelocities(LinearVelocity... velocity);

    public void update();



}
