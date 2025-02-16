package digilib.winch;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import digilib.MotorControllerType;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;

import static digilib.MotorControllerType.*;
import static edu.wpi.first.units.Units.Volts;

public class KrakenX60Winch implements Winch {
    private final WinchState state = new WinchState();
    private final TalonFX talonFX;
    private final WinchTelemetry telemetry;
    private WinchRequest winchRequest;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);

    public KrakenX60Winch(WinchConstants constants, TalonFX talonFX) {
        this.talonFX = talonFX;
        this.telemetry = new WinchTelemetry(constants.name());
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return TALONFX;
    }

    @Override
    public WinchState getState() {
        return state;
    }

    @Override
    public void setControl(WinchRequest request) {
        if (winchRequest != request) {
            winchRequest = request;
        }
        request.apply(this);
    }

    @Override
    public void setDutyCycle(Dimensionless dutyCycle) {
        talonFX.setControl(dutyCycleOut.withOutput(dutyCycle.baseUnitMagnitude()));
    }

    @Override
    public void setIdle() {
        talonFX.setControl(dutyCycleOut.withOutput(0.0));
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.withDutyCycle(talonFX.getDutyCycle().getValue())
                .withVoltage(talonFX.getMotorVoltage().getValue().in(Volts))
                .withTimestamp(Timer.getFPGATimestamp());
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {

    }
}
