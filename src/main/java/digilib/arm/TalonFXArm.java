package digilib.arm;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import digilib.MotorControllerType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

import static digilib.MotorControllerType.*;

public class TalonFXArm implements Arm {
    private final ArmState state = new ArmState();
    private final double minAngleRotations;
    private final double maxAngleRotations;
    private final double maxVelocityRPS;
    private final double reduction;
    private final ArmTelemetry telemetry;
    private final TalonFX talonFX;
    private final CANcoder cancoder;
    private final MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0.0).withSlot(0).withEnableFOC(true);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(1).withEnableFOC(true);
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private ArmRequest armRequest = null;
    private SimulatedArm simArm = null;
    private TalonFXSimState talonFXSimState = null;
    private CANcoderSimState canCoderSimState = null;

    public TalonFXArm(
            ArmConstants constants,
            TalonFX talonFX,
            CANcoder cancoder) {
        minAngleRotations = constants.minAngleDegrees() / 360.0;
        maxAngleRotations = constants.maxAngleDegrees() / 360.0;
        maxVelocityRPS = constants.maxVelocityRPS();
        reduction = constants.reduction();
        this.talonFX = talonFX;
        this.cancoder = cancoder;
        this.telemetry = new ArmTelemetry(
                constants.name(),
                constants.minAngleDegrees(),
                constants.maxAngleDegrees(),
                constants.maxVelocityRPS(),
                constants.maxAccelerationRPSSquared());

        if (RobotBase.isSimulation()) {
            canCoderSimState = new CANcoderSimState(cancoder);
            talonFXSimState = new TalonFXSimState(talonFX);
            simArm = SimulatedArm.createFromSysId(
                    constants.ksVolts(),
                    constants.kgVolts(),
                    constants.kvVoltsPerRPS(),
                    constants.kaVoltsPerRPSSquared(),
                    DCMotor.getKrakenX60Foc(1),
                    constants.reduction(),
                    constants.startingAngleDegrees(),
                    constants.minAngleDegrees(),
                    constants.maxAngleDegrees());
            canCoderSimState.setRawPosition(simArm.getAngleRads() / 2 / Math.PI);
            talonFXSimState.setRawRotorPosition(simArm.getAngleRads() * reduction / 2 / Math.PI);
        }
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return TALONFX;
    }

    @Override
    public double getMinAngleRotations() {
        return minAngleRotations;
    }

    @Override
    public double getMaxAngleRotations() {
        return maxAngleRotations;
    }

    @Override
    public double getMaxVelocityRPS() {
        return maxVelocityRPS;
    }

    @Override
    public ArmState getState() {
        return state;
    }

    @Override
    public void setControl(ArmRequest request) {
        if (armRequest != request) {
            armRequest = request;
        }
        request.apply(this);
    }

    @Override
    public void setPosition(double setpointRotations) {
        double currentPosition = talonFX.getPosition().getValueAsDouble();

        if (currentPosition > maxAngleRotations &&
                state.getMotorEncoderPositionRotations() > setpointRotations) {

        }

        if (state.getAngle().gte(arm.getMaxAngle()) && angle.gt(arm.getMaxAngle())) {
            arm.setPosition(angle.mut_replace(arm.getMaxAngle()));
        } else if (state.getAngle().lte(arm.getMinAngle()) && angle.lt(arm.getMinAngle())) {
            arm.setPosition(angle.mut_replace(arm.getMinAngle()));
        } else {
            arm.setPosition(angle);
        }
        talonFX.setControl(positionControl.withPosition(setpointRotations));
    }

    @Override
    public void setVelocity(double setpointScalar) {
        talonFX.setControl(velocityControl.withVelocity(setpointScalar));
    }

    @Override
    public void setVoltage(double volts) {
        talonFX.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void resetPosition() {
        talonFX.setPosition(0);
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    public void updateState() {
        state.setMotorEncoderPositionRotations(talonFX.getPosition().getValueAsDouble());
        state.setAbsoluteEncoderPositionRotations(cancoder.getAbsolutePosition().getValueAsDouble());
        state.setMotorEncoderVelocityRPS(talonFX.getVelocity().getValueAsDouble());
        state.setAbsoluteEncoderVelocityRPS(cancoder.getVelocity().getValueAsDouble());
        state.setVoltage(talonFX.getMotorVoltage().getValueAsDouble());
        state.setAbsoluteEncoderStatus(cancoder.getMagnetHealth().getValue());
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = talonFX.getMotorVoltage().getValue();
        simArm.setInputVoltage(inputVoltage.baseUnitMagnitude());
        simArm.update(dt);

        canCoderSimState.setSupplyVoltage(supplyVoltage);
        canCoderSimState.setMagnetHealth(MagnetHealthValue.Magnet_Green);
        canCoderSimState.setVelocity(simArm.getVelocityRadPerSec() / 2 / Math.PI);
        canCoderSimState.setRawPosition(simArm.getAngleRads() / 2 / Math.PI);

        talonFXSimState.setRawRotorPosition(simArm.getAngleRads() * reduction / 2 / Math.PI);
        talonFXSimState.setRotorVelocity(simArm.getVelocityRadPerSec() * reduction / 2 / Math.PI);
        talonFXSimState.setSupplyVoltage(supplyVoltage);
    }
}
