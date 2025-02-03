package frc.robot.mechanisms.winch;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Dimensionless;

public class KrakenX60Winch extends Winch {

    private final TalonFX talonFX;
    private final DutyCycleOut dutyCycleOut;
    private ControlState controlState;

    public KrakenX60Winch(
            TalonFX talonFX) {
        this.talonFX = talonFX;
        this.controlState = ControlState.IDLE;
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
    public void setDutyCycle(Dimensionless dutyCycle) {
        controlState = ControlState.DUTY_CYCLE;
        dutyCycleOut.withOutput(dutyCycle.baseUnitMagnitude());
    }

    @Override
    public void setIdle() {
        controlState = ControlState.IDLE;
        dutyCycleOut.withOutput(0.0);
    }

    @Override
    public void update() {
        super.update();
        applyDutyCycle();
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }

    private void applyDutyCycle() {
        talonFX.setControl(dutyCycleOut);
    }
}
