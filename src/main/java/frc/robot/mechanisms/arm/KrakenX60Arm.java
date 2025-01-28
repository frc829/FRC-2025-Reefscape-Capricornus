package frc.robot.mechanisms.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

public class KrakenX60Arm extends Arm {

    private final TalonFX talonFX;
    private final CANcoder canCoder;
    private final MotionMagicExpoVoltage positionControl;
    private final MotionMagicVelocityVoltage velocityControl;
    private ControlState controlState;

    public KrakenX60Arm(
            ArmControlParameters armControlParameters,
            TalonFX talonFX,
            CANcoder canCoder) {
        super(armControlParameters);
        this.talonFX = talonFX;
        this.canCoder = canCoder;
        this.controlState = ControlState.VELOCITY;
        this.positionControl = new MotionMagicExpoVoltage(0.0);
        this.velocityControl = new MotionMagicVelocityVoltage(0.0);
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
    public void setVelocity(AngularVelocity velocity) {
        // TODO: assign ControlState.VELOCITY to controlState
        // TODO: call velocityControl's withVelocity method and pass in velocity
    }

    @Override
    public void setPosition(Angle position) {
        // TODO: assign ControlState.VELOCITY to controlState
        // TODO: call positionControl's withPosition method and pass in position
    }

    @Override
    public void setHold() {
        // TODO: if the controlState is not equal to HOLD
        // TODO: then do the following
        // TODO: call positionControl's withPosition method and pass in talonFX.getPosition().getValue()
        // TODO: assign ControlState.HOLD to controlState

    }

    @Override
    public void resetPosition() {
        // TODO: will do later
    }

    @Override
    public void update() {
        super.update();
        switch (controlState) {
            case VELOCITY -> applyVelocity();
            case POSITION, HOLD -> applyPosition();
        }
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }

    private void applyVelocity() {
        // TODO: call talonFX's setControl method and pass in velocityControl
    }

    private void applyPosition() {
        // TODO: call talonFX's setControl method and pass in positionControl
    }
}
