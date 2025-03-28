package digilib.arm;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

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
    private SimulatedArm simArm = null;
    private TalonFXSimState talonFXSimState = null;
    private CANcoderSimState canCoderSimState = null;
    private MechanismLigament2d ligament = null;
    private double offsetDegrees = 0.0;



    public TalonFXArm(
            ArmConstants constants,
            TalonFX talonFX,
            CANcoder cancoder,
            MechanismLigament2d ligament,
            double offsetDegrees) {
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
                    constants.kvVoltsPerRPS() / 2 / Math.PI,
                    constants.kaVoltsPerRPSSquared() / 2 / Math.PI,
                    DCMotor.getKrakenX60Foc(1),
                    constants.reduction(),
                    constants.startingAngleDegrees() * Math.PI / 180,
                    constants.minAngleDegrees() * Math.PI / 180,
                    constants.maxAngleDegrees() * Math.PI / 180);
            canCoderSimState.setRawPosition(simArm.getAngleRads() / 2 / Math.PI);
            talonFXSimState.setRawRotorPosition(simArm.getAngleRads() * reduction / 2 / Math.PI);
            this.ligament = ligament;
            this.offsetDegrees = offsetDegrees;
        }
    }

    @Override
    public ArmState getState() {
        return state;
    }

    @Override
    public void setPosition(double setpointRotations) {
        double currentAngleRotations = talonFX.getPosition().getValueAsDouble();
        if (currentAngleRotations >= maxAngleRotations && setpointRotations > maxAngleRotations) {
            talonFX.setControl(positionControl.withPosition(maxAngleRotations));
        } else if (currentAngleRotations <= minAngleRotations && setpointRotations < minAngleRotations) {
            talonFX.setControl(positionControl.withPosition(minAngleRotations));
        } else {
            talonFX.setControl(positionControl.withPosition(setpointRotations));
        }
    }

    @Override
    public void setVelocity(double setpointScalar) {
        double velocitySetpointRPS = setpointScalar * maxVelocityRPS;
        double currentAngleRotations = talonFX.getPosition().getValueAsDouble();
        if (currentAngleRotations >= maxAngleRotations && setpointScalar > 0.0){
            talonFX.setControl(positionControl.withPosition(maxAngleRotations));
        }else if(currentAngleRotations <= minAngleRotations && setpointScalar < 0.0){
            talonFX.setControl(positionControl.withPosition(minAngleRotations));
        }else{
            talonFX.setControl(velocityControl.withVelocity(velocitySetpointRPS));
        }
    }

    @Override
    public void update() {
        state.setMotorEncoderPositionRotations(talonFX.getPosition().getValueAsDouble());
        state.setAbsoluteEncoderPositionRotations(cancoder.getPosition().getValueAsDouble());
        state.setMotorEncoderVelocityRPS(talonFX.getVelocity().getValueAsDouble());
        state.setAbsoluteEncoderVelocityRPS(cancoder.getVelocity().getValueAsDouble());
        state.setVolts(talonFX.getMotorVoltage().getValueAsDouble());
        state.setAmps(talonFX.getTorqueCurrent().getValueAsDouble());
        state.setAbsoluteEncoderStatus(cancoder.getMagnetHealth().getValue());
        telemetry.telemeterize(state);

        if(ligament!=null){
            ligament.setAngle(offsetDegrees + state.getAbsoluteEncoderPositionDegrees());
        }
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
