package digilib.winch;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.*;

public class KrakenX60Winch implements Winch {
    private final WinchState lastWinchState = new WinchState();
    private final WinchState winchState = new WinchState();
    private WinchRequest winchRequest;
    private final TalonFX talonFX;
    private final Distance drumRadius;
    private final DutyCycleOut dutyCycleOut;
    private final MutDimensionless dutyCycle = Value.mutable(0.0);
    private final MutDistance position = Meters.mutable(0.0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private boolean hold = false;

    public KrakenX60Winch(TalonFX talonFX, Distance drumRadius) {
        this.talonFX = talonFX;
        this.drumRadius = drumRadius;
        this.dutyCycleOut = new DutyCycleOut(0.0);
    }

    @Override
    public boolean setNeutralModeToBrake() {
        StatusCode code = talonFX.setNeutralMode(NeutralModeValue.Brake);
        return code == StatusCode.OK;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        StatusCode code = talonFX.setNeutralMode(NeutralModeValue.Coast);
        return code == StatusCode.OK;
    }

    @Override
    public WinchState getState() {
        return winchState;
    }

    @Override
    public WinchState getStateCopy() {
        return winchState.clone();
    }

    @Override
    public WinchState getLastArmState() {
        return lastWinchState;
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
        talonFX.setControl(dutyCycleOut);
    }

    @Override
    public void update() {
        lastWinchState.withWinchState(winchState);
        updateState();
        updateTelemetry();
    }

    private void updateState() {
        winchState.withDutyCycle(dutyCycle.mut_setMagnitude(talonFX.getDutyCycle().getValue()));
        winchState.withTimestamp(timestamp.mut_setMagnitude(Timer.getFPGATimestamp()));
        winchState.withPosition(position.mut_setMagnitude(talonFX.getPosition().getValue().baseUnitMagnitude() * drumRadius.in(Meters)));
        winchState.withVelocity(velocity.mut_setMagnitude(talonFX.getVelocity().getValue().baseUnitMagnitude() * drumRadius.in(Meters)));
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {

    }

    @Override
    public void enableHold() {
        hold = true;
    }

    @Override
    public void disableHold() {
        hold = false;
    }

    @Override
    public boolean isHoldEnabled() {
        return hold;
    }
}
