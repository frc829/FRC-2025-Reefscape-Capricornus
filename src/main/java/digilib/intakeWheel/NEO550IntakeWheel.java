package digilib.intakeWheel;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import digilib.MotorControllerType;
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
import static digilib.MotorControllerType.*;
import static digilib.intakeWheel.NEO550IntakeWheel.ControlState.VELOCITY;
import static digilib.intakeWheel.NEO550IntakeWheel.ControlState.VOLTAGE;
import static edu.wpi.first.units.Units.*;

public class NEO550IntakeWheel implements IntakeWheel {

    public enum ControlState {
        VELOCITY,
        VOLTAGE
    }

    private final IntakeWheelState state = new IntakeWheelState();
    private final double maxVelocityRPS;
    private final IntakeWheelTelemetry telemetry;
    private final SparkMax motor;
    private ControlState controlState = null;
    private final SlewRateLimiter profile;
    private final SimpleMotorFeedforward feedforward;
    private final MutAngularVelocity setpoint = RotationsPerSecond.mutable(0.0);
    private FlywheelSim flywheelSim = null;
    private SparkMaxSim sparkMaxSim = null;

    public NEO550IntakeWheel(
            IntakeWheelConstants constants,
            SparkMax motor,
            double controlPeriodSeconds) {
        maxVelocityRPS = constants.maxVelocityRPS();
        this.motor = motor;
        this.telemetry = new IntakeWheelTelemetry(
                constants.name(),
                constants.maxVelocityRPS(),
                constants.maxAccelerationRPSSquared());
        this.feedforward = new SimpleMotorFeedforward(
                constants.ksVolts(),
                constants.kvVoltsPerRPS(),
                constants.kaVoltsPerRPSSquared(),
                controlPeriodSeconds);
        this.profile = new SlewRateLimiter(constants.maxAccelerationRPSSquared());

        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeo550(1);
            sparkMaxSim = new SparkMaxSim(motor, dcMotor);
            LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(
                    constants.kvVoltsPerRPS(),
                    constants.kaVoltsPerRPSSquared());
            flywheelSim = new FlywheelSim(
                    plant,
                    dcMotor);
        }
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return REV_SPARK_MAX;
    }

    @Override
    public double getMaxVelocityRPS() {
        return maxVelocityRPS;
    }

    @Override
    public IntakeWheelState getState() {
        return state;
    }

    @Override
    public void setVelocity(double setpointScalar) {
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
    public void setVoltage(double volts) {
        if (controlState != VOLTAGE) {
            controlState = VOLTAGE;
        }
        motor.setVoltage(volts);
        }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.setMotorEncoderVelocityRPS(motor.getEncoder().getVelocity());
        state.setVolts(motor.getAppliedOutput() * motor.getBusVoltage());
        state.setAmps(motor.getOutputCurrent());
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        double inputVoltage = motor.getAppliedOutput() * 12.0;
        flywheelSim.setInputVoltage(inputVoltage);
        flywheelSim.update(dtSeconds);
        sparkMaxSim.iterate(flywheelSim.getOutput(0), 12.0, dtSeconds);
    }
}
