package frc.robot.mechanisms.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Seconds;

public class KrakenX60Arm implements Arm {

    private final ArmState lastArmState = new ArmState();
    private final ArmState armState = new ArmState();
    private final Angle minAngle;
    private final Angle maxAngle;
    private ArmRequest armRequest;
    private final TalonFX talonFX;
    private final CANcoder canCoder;
    private final PositionVoltage positionControl;
    private final MotionMagicVelocityVoltage velocityControl;
    private final SimArm simArm;
    private final TalonFXSimState talonFXSimState;
    private final CANcoderSimState canCoderSimState;
    private final MutTime timeStamp = Seconds.mutable(0.0);
    private ControlState controlState = ControlState.VELOCITY;

    public KrakenX60Arm(
            ArmConstants armConstants,
            TalonFX talonFX,
            CANcoder canCoder) {
        this.minAngle = armConstants.getMinAngle();
        this.maxAngle = armConstants.getMaxAngle();
        this.talonFX = talonFX;
        this.talonFXSimState = new TalonFXSimState(talonFX);
        this.canCoder = canCoder;
        this.canCoderSimState = new CANcoderSimState(canCoder);
        this.positionControl = new PositionVoltage(0.0).withSlot(0);
        this.velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(1);
        this.simArm = new SimArm(
                DCMotor.getKrakenX60Foc(1),
                armConstants.getReduction(),
                armConstants.getKs(),
                armConstants.getKg(),
                armConstants.getKv(),
                armConstants.getKa(),
                armConstants.getArmLength(),
                armConstants.getMinAngle(),
                armConstants.getMaxAngle(),
                armConstants.getStartingAngle(),
                armConstants.getPositionStdDev(),
                armConstants.getVelocityStdDev());
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
    public ArmState getState() {
        return armState;
    }

    @Override
    public ArmState getStateCopy() {
        return armState.clone();
    }

    @Override
    public ArmState getLastArmState() {
        return lastArmState;
    }

    @Override
    public ArmRequest createHoldRequest() {
        return new ArmRequest.Hold();
    }

    @Override
    public ArmRequest createPositionRequest() {
        return new ArmRequest.Position(minAngle, maxAngle);
    }

    @Override
    public ArmRequest createVelocityRequest() {
        return new ArmRequest.Velocity(minAngle, maxAngle);
    }

    @Override
    public void setControl(ArmRequest request) {
        if(armRequest != request){
            armRequest = request;
        }
        request.apply(this);
    }

    @Override
    public void setPosition(Angle position) {
        controlState = ControlState.POSITION;
        positionControl.withPosition(position);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        controlState = ControlState.VELOCITY;
        velocityControl.withVelocity(velocity);
    }

    @Override
    public void setHold() {
        if (controlState != ControlState.HOLD) {
            positionControl.withPosition(talonFX.getPosition().getValue());
            controlState = ControlState.HOLD;
        }
    }

    @Override
    public void resetPosition() {
        if(canCoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Invalid && canCoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red){
            talonFX.setPosition(canCoder.getPosition().getValue());
        }
        updateState();
    }

    @Override
    public void update() {
        lastArmState.withArmState(armState);
        updateState();
        updateTelemetry();
        switch (controlState) {
            case VELOCITY -> applyVelocity();
            case POSITION, HOLD -> applyPosition();
        }
    }

    private void updateState() {
        armState.withPosition(talonFX.getPosition().getValue());
        armState.withVelocity(talonFX.getVelocity().getValue());
        armState.withTimestamp(timeStamp.mut_setMagnitude(Timer.getFPGATimestamp()));
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }

    @Override
    public void updateSimState(Time dt, Voltage supplyVoltage) {
        var inputVoltage = talonFX.getMotorVoltage().getValue();
        SmartDashboard.putNumber("Input Voltage", inputVoltage.baseUnitMagnitude());
        simArm.update(dt, inputVoltage);
        talonFXSimState.setRawRotorPosition(simArm.getRotorAngle());
        talonFXSimState.setRotorVelocity(simArm.getRotorVelocity());
        talonFXSimState.setRotorAcceleration(simArm.getRotorAcceleration());
        talonFXSimState.setSupplyVoltage(12.0);
        canCoderSimState.setSupplyVoltage(12.0);
        canCoderSimState.setMagnetHealth(MagnetHealthValue.Magnet_Green);
        canCoderSimState.setVelocity(simArm.getAngularVelocity());
        canCoderSimState.setRawPosition(simArm.getAngle());
    }

    private void applyVelocity() {
        talonFX.setControl(velocityControl);
    }

    private void applyPosition() {
        SmartDashboard.putNumber("Applied Position", positionControl.Position);
        talonFX.setControl(positionControl);
    }
}
