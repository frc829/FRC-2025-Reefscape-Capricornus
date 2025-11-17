package digilib.intakeWheel;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TalonFXSIntakeWheel extends IntakeWheel {
    private final double maxVelocityRPS;
    private final TalonFXS motor;
    private final double reduction;
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private final DCMotorSim simIntakeWheel;
    private final TalonFXSSimState talonFXSSimState;

    public TalonFXSIntakeWheel(
            final String name,
            final double maxVelocityRPS,
            final double maxAccelerationRPSSquared,
            final TalonFXS motor,
            final MotionMagicVelocityVoltage motionMagicVelocityVoltage,
            final double reduction,
            final DCMotorSim simIntakeWheel,
            final TalonFXSSimState talonFXSSimState) {
        super(name, maxVelocityRPS, maxAccelerationRPSSquared);

        this.maxVelocityRPS = maxVelocityRPS;
        this.motor = motor;
        this.motionMagicVelocityVoltage = motionMagicVelocityVoltage;
        this.reduction = reduction;
        this.simIntakeWheel = simIntakeWheel;
        this.talonFXSSimState = talonFXSSimState;

    }

    @Override
    public double getMotorEncoderVelocityDPS() {
        return motor.getVelocity().getValueAsDouble() * 360.0;
    }

    @Override
    public void applyMotorEncoderVelocity(double goalScalar) {
        double velocitySetpointRPS = goalScalar * maxVelocityRPS;
        motor.setControl(motionMagicVelocityVoltage.withVelocity(velocitySetpointRPS));
    }

    @Override
    public double getVolts() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public void applyVolts(double volts){
        motor.setVoltage(volts);
    }

    @Override
    public double getAmps() {
        return motor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        var inputVoltage = motor.getMotorVoltage().getValue();
        simIntakeWheel.setInputVoltage(inputVoltage.baseUnitMagnitude());
        simIntakeWheel.update(dtSeconds);

        talonFXSSimState.setRawRotorPosition(simIntakeWheel.getAngularPositionRad() * reduction / 2 / Math.PI);
        talonFXSSimState.setRotorVelocity(simIntakeWheel.getAngularVelocityRadPerSec() * reduction / 2 / Math.PI);
        talonFXSSimState.setSupplyVoltage(supplyVoltage);
    }
}
