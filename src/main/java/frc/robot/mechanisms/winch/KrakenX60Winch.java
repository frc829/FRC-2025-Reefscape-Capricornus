package frc.robot.mechanisms.winch;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
        // TODO: call talonFX.setNeutralMode() passing in NeutralModeValue.Brake and assign to a StatusCode variable called code.
        // TODO: return code == StatusCode.OK
        return false; // TODO: remove this when done.
    }

    @Override
    public boolean setNeutralModeToCoast() {
        // TODO: call talonFX.setNeutralMode() passing in NeutralModeValue.Coast and assign to a StatusCode variable called code.
        // TODO: return code == StatusCode.OK
        return false; // TODO: remove this when done.
    }

    @Override
    public void setDutyCycle(Dimensionless dutyCycle) {
        // TODO: assign ControlState.DUTY_CYCLE to controlState
        // TODO: call dutyCycleOut's withOutput method and pass in dutyCycle.baseUnitMagnitude()
    }

    @Override
    public void setIdle() {
        // TODO: assign ControlState.IDLE to controlState
        // TODO: call dutyCycleOut's withOutput method and pass in 0.0

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
        // TODO: call talonFX's setControl method and pass in dutyCycleControl
    }
}
