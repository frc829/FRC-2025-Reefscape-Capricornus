package digilib.intakeWheel;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static com.revrobotics.spark.ClosedLoopSlot.*;
import static com.revrobotics.spark.SparkBase.ControlType.*;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;

public class SparkMaxIntakeWheel extends IntakeWheel {
    private final double maxVelocityRPS;
    private final SparkMax motor;
    private final SlewRateLimiter profile;
    private final SimpleMotorFeedforward feedforward;
    private final DCMotorSim flywheelSim;
    private final SparkMaxSim sparkMaxSim;
    private double setpoint;


    public SparkMaxIntakeWheel(
            final String name,
            final double maxVelocityRPS,
            final double maxAccelerationRPSSquared,
            final SparkMax motor,
            final SlewRateLimiter profile,
            final SimpleMotorFeedforward feedforward,
            final DCMotorSim flywheelSim,
            final SparkMaxSim sparkMaxSim) {
        super(name, maxVelocityRPS, maxAccelerationRPSSquared);

        this.maxVelocityRPS = maxVelocityRPS;
        this.motor = motor;
        this.profile = profile;
        this.feedforward = feedforward;
        this.flywheelSim = flywheelSim;
        this.sparkMaxSim = sparkMaxSim;
    }

    @Override
    public double getMotorEncoderVelocityDPS() {
        return motor.getEncoder().getVelocity() * 360.0;
    }

    @Override
    public void applyMotorEncoderVelocity(double goalScalar) {
        double goalVelocity = goalScalar * maxVelocityRPS;
        double nextVelocitySetpoint = profile.calculate(goalVelocity);
        double arbFeedforward = feedforward.calculateWithVelocities(setpoint, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(
                nextVelocitySetpoint,
                kVelocity,
                kSlot1,
                arbFeedforward,
                kVoltage);
        setpoint = nextVelocitySetpoint;
    }

    @Override
    public double getVolts() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public void applyVolts(double volts) {
        motor.setVoltage(volts);
        setpoint = motor.getEncoder().getVelocity();
        profile.reset(setpoint);
    }

    @Override
    public double getAmps() {
        return motor.getOutputCurrent();
    }

    @Override
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        if (flywheelSim != null && sparkMaxSim != null) {
            double inputVoltage = motor.getAppliedOutput() * supplyVoltage;
            flywheelSim.setInputVoltage(inputVoltage);
            flywheelSim.update(dtSeconds);
            sparkMaxSim.iterate(flywheelSim.getOutput(0), supplyVoltage, dtSeconds);
        }
    }
}
