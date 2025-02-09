package digilib.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class KrakenX60Arm implements Arm {
    private final ArmState lastArmState = new ArmState();
    private final ArmState armState = new ArmState();
    private final Angle minAngle;
    private final Angle maxAngle;
    private final AngularVelocity maxAngularVelocity;
    private final ArmTelemetry armTelemetry;
    private ArmRequest armRequest;
    private final ArmConstants armConstants;
    private final TalonFX talonFX;
    private final CANcoder canCoder;
    private final MotionMagicExpoVoltage positionControl;
    private final MotionMagicVelocityVoltage velocityControl;
    private final SingleJointedArmSim simArm;
    private final TalonFXSimState talonFXSimState;
    private final CANcoderSimState canCoderSimState;
    private final MutTime timeStamp = Seconds.mutable(0.0);
    private boolean hold = false;

    public KrakenX60Arm(
            ArmConstants armConstants,
            TalonFX talonFX,
            CANcoder canCoder) {
        this.minAngle = armConstants.getMinAngle();
        this.maxAngle = armConstants.getMaxAngle();
        this.maxAngularVelocity = armConstants.getMaxAngularVelocity();
        this.armConstants = armConstants;
        this.talonFX = talonFX;
        this.talonFXSimState = new TalonFXSimState(talonFX);
        this.canCoder = canCoder;
        this.canCoderSimState = new CANcoderSimState(canCoder);
        this.positionControl = new MotionMagicExpoVoltage(0.0).withSlot(0).withEnableFOC(true);
        this.velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(1).withEnableFOC(true);
        LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(armConstants.getKv().baseUnitMagnitude(), armConstants.getKa().baseUnitMagnitude());
        this.simArm = new SingleJointedArmSim(
                plant,
                DCMotor.getKrakenX60Foc(1),
                armConstants.getReduction(),
                armConstants.getArmLength().baseUnitMagnitude(),
                armConstants.getMinAngle().baseUnitMagnitude(),
                armConstants.getMaxAngle().baseUnitMagnitude(),
                true,
                Radians.of(0.0).baseUnitMagnitude());
        this.armTelemetry = new ArmTelemetry(
                armConstants.getName(),
                armConstants.getMinAngle(),
                armConstants.getMaxAngle(),
                armConstants.getMaxAngularVelocity(),
                armConstants.getMaxAngularAcceleration()
        );
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
    public ArmState getLastState() {
        return lastArmState;
    }

    @Override
    public void setControl(ArmRequest request) {
        if (armRequest != request) {
            armRequest = request;
        }
        request.apply(this);
    }

    @Override
    public void setPosition(Angle position) {
        talonFX.setControl(positionControl.withPosition(position));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        talonFX.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void resetPosition() {
        if (canCoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Invalid && canCoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red) {
            talonFX.setPosition(canCoder.getPosition().getValue());
        }
        updateState();
    }

    @Override
    public void update() {
        lastArmState.withArmState(armState);
        updateState();
        updateTelemetry();
    }

    private void updateState() {
        armState.withPosition(talonFX.getPosition().getValue())
                .withVelocity(talonFX.getVelocity().getValue())
                .withAbsolutePosition(canCoder.getAbsolutePosition().getValue())
                .withAbsoluteVelocity(canCoder.getVelocity().getValue())
                .withTimestamp(timeStamp.mut_setMagnitude(Timer.getFPGATimestamp()));
    }

    @Override
    public void updateTelemetry() {
        armTelemetry.telemeterize(armState);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = talonFX.getMotorVoltage().getValue();
        simArm.setInputVoltage(inputVoltage.baseUnitMagnitude());
        simArm.update(dt);

        talonFXSimState.setRawRotorPosition(simArm.getAngleRads() * armConstants.getReduction() / 2 / Math.PI);
        talonFXSimState.setRotorVelocity(simArm.getVelocityRadPerSec() * armConstants.getReduction() / 2 / Math.PI);

        talonFXSimState.setSupplyVoltage(supplyVoltage);
        canCoderSimState.setSupplyVoltage(supplyVoltage);
        canCoderSimState.setMagnetHealth(MagnetHealthValue.Magnet_Green);
        canCoderSimState.setVelocity(simArm.getVelocityRadPerSec() / 2 / Math.PI);
        canCoderSimState.setRawPosition(simArm.getAngleRads() / 2 / Math.PI);
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

    @Override
    public Angle getMinAngle() {
        return minAngle;
    }

    @Override
    public Angle getMaxAngle() {
        return maxAngle;
    }

    @Override
    public AngularVelocity getMaxVelocity() {
        return maxAngularVelocity;
    }
}
