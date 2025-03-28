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

import static edu.wpi.first.units.Units.*;

public class TalonFXArm extends Arm {
    private final double minAngleRotations;
    private final double maxAngleRotations;
    private final double maxVelocityRPS;
    private final double reduction;
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
            Config config,
            TalonFX talonFX,
            CANcoder cancoder,
            MechanismLigament2d ligament,
            double offsetDegrees) {
        super(
                config.name(),
                config.minAngleDegrees(),
                config.maxAngleDegrees(),
                config.maxVelocityRPS(),
                config.maxAccelerationRPSSquared());
        minAngleRotations = config.minAngleDegrees() / 360.0;
        maxAngleRotations = config.maxAngleDegrees() / 360.0;
        maxVelocityRPS = config.maxVelocityRPS();
        reduction = config.reduction();
        this.talonFX = talonFX;
        this.cancoder = cancoder;

        if (RobotBase.isSimulation()) {
            canCoderSimState = new CANcoderSimState(cancoder);
            talonFXSimState = new TalonFXSimState(talonFX);
            simArm = SimulatedArm.createFromSysId(
                    config.ksVolts(),
                    config.kgVolts(),
                    config.kvVoltsPerRPS() / 2 / Math.PI,
                    config.kaVoltsPerRPSSquared() / 2 / Math.PI,
                    DCMotor.getKrakenX60Foc(1),
                    config.reduction(),
                    config.startingAngleDegrees() * Math.PI / 180,
                    config.minAngleDegrees() * Math.PI / 180,
                    config.maxAngleDegrees() * Math.PI / 180);
            canCoderSimState.setRawPosition(simArm.getAngleRads() / 2 / Math.PI);
            talonFXSimState.setRawRotorPosition(simArm.getAngleRads() * reduction / 2 / Math.PI);
            this.ligament = ligament;
            this.offsetDegrees = offsetDegrees;
        }
    }

    @Override
    public double getMotorEncoderPositionDegrees() {
        return talonFX.getPosition().getValue().in(Degrees);
    }

    @Override
    public double getMotorEncoderVelocityDPS() {
        return talonFX.getVelocity().getValue().in(DegreesPerSecond);
    }

    @Override
    public double getAbsoluteEncoderPositionRotations() {
        return cancoder.getAbsolutePosition().getValue().in(Rotations);
    }

    @Override
    public double getAbsoluteEncoderPositionDegrees() {
        return cancoder.getAbsolutePosition().getValue().in(Degrees);
    }

    @Override
    public double getAbsoluteEncoderVelocityDPS() {
        return cancoder.getVelocity().getValue().in(DegreesPerSecond);
    }

    @Override
    public double getVolts() {
        return talonFX.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getAmps() {
        return talonFX.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public String getAbsoluteEncoderStatus() {
        return AbsoluteEncoderStatus.fromMagnetHealthValue(cancoder.getMagnetHealth().getValue()).name();
    }


    @Override
    public void applyPosition(double setpointRotations) {
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
    public void applyVelocity(double setpointScalar) {
        double velocitySetpointRPS = setpointScalar * maxVelocityRPS;
        double currentAngleRotations = talonFX.getPosition().getValueAsDouble();
        if (currentAngleRotations >= maxAngleRotations && setpointScalar > 0.0) {
            talonFX.setControl(positionControl.withPosition(maxAngleRotations));
        } else if (currentAngleRotations <= minAngleRotations && setpointScalar < 0.0) {
            talonFX.setControl(positionControl.withPosition(minAngleRotations));
        } else {
            talonFX.setControl(velocityControl.withVelocity(velocitySetpointRPS));
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
        ligament.setAngle(offsetDegrees + getAbsoluteEncoderPositionDegrees());
    }
}
