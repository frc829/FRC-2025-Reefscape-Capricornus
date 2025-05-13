package digilib.wrist;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class TalonFXSWrist extends Wrist {

    private final double minAngleRotations;
    private final double maxAngleRotations;
    private final double maxVelocityRPS;
    private final TalonFXS motor;
    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private final double reduction;
    private DCMotorSim simWrist = null;
    private TalonFXSSimState sim = null;
    private MechanismLigament2d top = null;
    private MechanismLigament2d bottom = null;

    public TalonFXSWrist(
            String name,
            double minAngleDegrees,
            double maxAngleDegrees,
            double maxVelocityRPS,
            double maxAccelerationRPSSquared,
            double reduction,
            double startingAngleDegrees,
            TalonFXS motor,
            MotionMagicExpoVoltage motionMagicExpoVoltage,
            MotionMagicVelocityVoltage motionMagicVelocityVoltage,
            DCMotorSim simWrist,
            TalonFXSSimState sim,
            MechanismLigament2d top,
            MechanismLigament2d bottom) {
        super(
                name,
                minAngleDegrees,
                maxAngleDegrees,
                maxVelocityRPS,
                maxAccelerationRPSSquared);
        this.reduction = reduction;
        minAngleRotations = minAngleDegrees / 360.0;
        maxAngleRotations = maxAngleDegrees / 360.0;
        this.maxVelocityRPS = maxVelocityRPS;
        this.motor = motor;
        this.motionMagicExpoVoltage = motionMagicExpoVoltage;
        this.motionMagicVelocityVoltage = motionMagicVelocityVoltage;
        motor.setPosition(0.0);

        if (RobotBase.isSimulation()) {
            this.simWrist = simWrist;
            this.sim = sim;
            simWrist.setAngle(startingAngleDegrees / 360.0);
            sim.setRawRotorPosition(startingAngleDegrees / 360.0);
            this.top = top;
            this.bottom = bottom;
        }
    }

    @Override
    public double getMotorEncoderPositionRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getMotorEncoderPositionDegrees() {
        return getMotorEncoderPositionRotations() * 360.0;
    }

    @Override
    public double getMotorEncoderVelocityDPS() {
        return motor.getVelocity().getValueAsDouble() * 360.0;
    }

    @Override
    public void applyPositionRotations(double setpointRotations) {
        double currentAngleRotations = motor.getPosition().getValueAsDouble();
        if (currentAngleRotations >= maxAngleRotations && setpointRotations > maxAngleRotations) {
            motor.setControl(motionMagicExpoVoltage.withPosition(maxAngleRotations));
        } else if (currentAngleRotations <= minAngleRotations && setpointRotations < minAngleRotations) {
            motor.setControl(motionMagicExpoVoltage.withPosition(minAngleRotations));
        } else {
            motor.setControl(motionMagicExpoVoltage.withPosition(setpointRotations));
        }
    }

    @Override
    public void applyVelocity(double setpointScalar) {
        double velocitySetpointRPS = setpointScalar * maxVelocityRPS;
        double currentAngleRotations = motor.getPosition().getValueAsDouble();
        if (currentAngleRotations >= maxAngleRotations && setpointScalar > 0.0) {
            motor.setControl(motionMagicExpoVoltage.withPosition(maxAngleRotations));
        } else if (currentAngleRotations <= minAngleRotations && setpointScalar < 0.0) {
            motor.setControl(motionMagicExpoVoltage.withPosition(minAngleRotations));
        } else {
            motor.setControl(motionMagicVelocityVoltage.withVelocity(velocitySetpointRPS));
        }    }

    @Override
    public double getVolts() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getAmps() {
        return motor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getMotorVoltage().getValueAsDouble();
        simWrist.setInputVoltage(inputVoltage);
        simWrist.update(dt);
        sim.setRawRotorPosition(simWrist.getAngularPositionRad() / 2 / Math.PI * reduction);
        sim.setRotorVelocity(simWrist.getAngularVelocityRadPerSec() / 2 / Math.PI * reduction);
        sim.setRotorAcceleration(simWrist.getAngularAccelerationRadPerSecSq() / 2 / Math.PI * reduction);
        top.setLength(0.3 * Math.cos(Math.toRadians(getMotorEncoderPositionDegrees())));
        bottom.setLength(0.3 * Math.cos(Math.toRadians(getMotorEncoderPositionDegrees())));
    }
}
