package digilib.winch;

import digilib.MotorControllerType;
import edu.wpi.first.units.measure.Dimensionless;

public interface Winch {

    MotorControllerType getMotorControllerType();

    WinchState getState();

    void setControl(WinchRequest request);

    void setDutyCycle(Dimensionless dutyCycle);

    void setIdle();

    void update();

    void updateState();

    void updateTelemetry();

    void updateSimState(double dt, double supplyVoltage);


}
