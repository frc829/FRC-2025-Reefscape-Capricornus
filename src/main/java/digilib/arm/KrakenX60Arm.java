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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static digilib.MotorControllerType.*;
import static edu.wpi.first.units.Units.*;

public class KrakenX60Arm implements Arm {
    private final ArmState state = new ArmState();
    private final Angle minAngle;
    private final Angle maxAngle;
    private final AngularVelocity maxAngularVelocity;
    private final double reduction;
    private final ArmTelemetry telemetry;
    private final TalonFX talonFX;
    private final CANcoder cancoder;
    private final MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0.0).withSlot(0).withEnableFOC(true);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(1).withEnableFOC(true);
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private ArmRequest armRequest;
    private boolean hold = false;
    private SingleJointedArmSim simArm = null;
    private TalonFXSimState talonFXSimState = null;
    private CANcoderSimState canCoderSimState = null;

    public KrakenX60Arm(
            ArmConstants constants,
            TalonFX talonFX,
            CANcoder cancoder) {
        minAngle = constants.minAngle();
        maxAngle = constants.maxAngle();
        maxAngularVelocity = constants.maxAngularVelocity();
        reduction = constants.reduction();
        this.talonFX = talonFX;
        this.cancoder = cancoder;
        this.telemetry = new ArmTelemetry(
                constants.name(),
                constants.minAngle(),
                constants.maxAngle(),
                constants.maxAngularVelocity(),
                constants.maxAngularAcceleration());
        talonFX.setPosition(cancoder.getAbsolutePosition().getValue());

        if (RobotBase.isSimulation()) {
            canCoderSimState = new CANcoderSimState(cancoder);
            talonFXSimState = new TalonFXSimState(talonFX);
            LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(constants.kv().baseUnitMagnitude(), constants.ka().baseUnitMagnitude());
            simArm = new SingleJointedArmSim(
                    plant,
                    DCMotor.getKrakenX60Foc(1),
                    constants.reduction(),
                    constants.armLength().baseUnitMagnitude(),
                    constants.minAngle().baseUnitMagnitude(),
                    constants.maxAngle().baseUnitMagnitude(),
                    true,
                    constants.startingAngle().baseUnitMagnitude());
            canCoderSimState.setRawPosition(simArm.getAngleRads() / 2 / Math.PI);
            talonFXSimState.setRawRotorPosition(simArm.getAngleRads() * reduction / 2 / Math.PI);
        }
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return TALONFX;
    }

    @Override
    public Angle getMaxAngle() {
        return maxAngle;
    }

    @Override
    public Angle getMinAngle() {
        return minAngle;
    }

    @Override
    public AngularVelocity getMaxVelocity() {
        return maxAngularVelocity;
    }

    @Override
    public ArmState getState() {
        return state;
    }

    @Override
    public boolean isHoldEnabled() {
        return hold;
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
    public void setVoltage(Voltage voltage) {
        talonFX.setControl(voltageOut.withOutput(voltage));
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
    public void resetPosition() {
        if (cancoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Invalid && cancoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Red) {
            talonFX.setPosition(cancoder.getAbsolutePosition().getValue());
        }
        updateState();
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    public void updateState() {
        state.withPosition(talonFX.getPosition().getValue().in(Radians))
                .withAbsolutePosition(cancoder.getAbsolutePosition().getValue().in(Radians))
                .withVelocity(talonFX.getVelocity().getValue().in(RadiansPerSecond))
                .withAbsoluteVelocity(cancoder.getVelocity().getValue().in(RadiansPerSecond))
                .withVoltage(talonFX.getMotorVoltage().getValueAsDouble())
                .withTimestamp(Timer.getFPGATimestamp())
                .withAbsoluteEncoderStatus(cancoder.getMagnetHealth().getValue().name());
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
