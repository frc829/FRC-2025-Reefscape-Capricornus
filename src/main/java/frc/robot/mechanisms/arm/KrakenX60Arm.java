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
        StatusCode code = talonFX.setNeutralMode(NeutralModeValue.Brake);
        return code == StatusCode.OK;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        StatusCode code = talonFX.setNeutralMode(NeutralModeValue.Coast);
        return code == StatusCode.OK;
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        controlState = ControlState.VELOCITY;
        velocityControl.withVelocity(velocity);
    }

    @Override
    public void setPosition(Angle position) {
        controlState = ControlState.VELOCITY;
        positionControl.withPosition(position);
    }

    @Override
    public void setHold() {
        if(controlState != ControlState.HOLD) {
            positionControl.withPosition(talonFX.getPosition().getValue());
            controlState = ControlState.HOLD;
        }
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
        talonFX.setControl(velocityControl);
    }

    private void applyPosition() {
        talonFX.setControl(positionControl);
    }
}
