package digilib.intakeWheel;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static com.revrobotics.spark.ClosedLoopSlot.*;
import static com.revrobotics.spark.SparkBase.ControlType.*;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;
import static digilib.intakeWheel.NEO550IntakeWheel.ControlState.VELOCITY;
import static digilib.intakeWheel.NEO550IntakeWheel.ControlState.VOLTAGE;
import static edu.wpi.first.units.Units.*;

public class NEO550IntakeWheel extends IntakeWheel {

    public enum ControlState {
        VELOCITY,
        VOLTAGE
    }

    private final double maxVelocityRPS;
    private final SparkMax motor;
    private ControlState controlState = null;
    private final SlewRateLimiter profile;
    private final SimpleMotorFeedforward feedforward;
    private final MutAngularVelocity setpoint = RotationsPerSecond.mutable(0.0);
    private FlywheelSim flywheelSim = null;
    private SparkMaxSim sparkMaxSim = null;

    public NEO550IntakeWheel(
            Config config,
            SparkMax motor,
            double controlPeriodSeconds) {
        super(config.name(),
                config.maxVelocityRPS(),
                config.maxAccelerationRPSSquared());
        maxVelocityRPS = config.maxVelocityRPS();
        this.motor = motor;
        this.feedforward = new SimpleMotorFeedforward(
                config.ksVolts(),
                config.kvVoltsPerRPS(),
                config.kaVoltsPerRPSSquared(),
                controlPeriodSeconds);
        this.profile = new SlewRateLimiter(config.maxAccelerationRPSSquared());

        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeo550(1);
            sparkMaxSim = new SparkMaxSim(motor, dcMotor);
            LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(
                    config.kvVoltsPerRPS(),
                    config.kaVoltsPerRPSSquared());
            flywheelSim = new FlywheelSim(
                    plant,
                    dcMotor);
        }
    }

    @Override
    public double getMotorEncoderVelocityDPS() {
        return motor.getEncoder().getVelocity() * 360.0;
    }

    @Override
    public void applyMotorEncoderVelocity(double setpointScalar) {
        if (controlState != VELOCITY) {
            profile.reset(motor.getEncoder().getVelocity());
            setpoint.mut_setMagnitude(motor.getEncoder().getVelocity());
            controlState = VELOCITY;
        }
        double goalVelocity = setpointScalar * maxVelocityRPS;
        double nextVelocitySetpoint = profile.calculate(goalVelocity);
        double arbFeedforward = feedforward.calculateWithVelocities(setpoint.in(RotationsPerSecond), nextVelocitySetpoint);
        motor.getClosedLoopController()
                .setReference(
                        nextVelocitySetpoint,
                        kVelocity,
                        kSlot1,
                        arbFeedforward,
                        kVoltage);
        setpoint.mut_setMagnitude(nextVelocitySetpoint);
    }

    @Override
    public double getVolts() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public void applyVolts(double volts) {
        if (controlState != VOLTAGE) {
            controlState = VOLTAGE;
        }
        motor.setVoltage(volts);
    }

    @Override
    public double getAmps() {
        return motor.getOutputCurrent();
    }

    @Override
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        double inputVoltage = motor.getAppliedOutput() * supplyVoltage;
        flywheelSim.setInputVoltage(inputVoltage);
        flywheelSim.update(dtSeconds);
        sparkMaxSim.iterate(flywheelSim.getOutput(0), supplyVoltage, dtSeconds);
    }
}
