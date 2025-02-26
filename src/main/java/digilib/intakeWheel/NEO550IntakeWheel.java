package digilib.intakeWheel;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static digilib.MotorControllerType.*;
import static edu.wpi.first.units.Units.*;

public class NEO550IntakeWheel implements IntakeWheel {
    private final IntakeWheelState state = new IntakeWheelState();
    private final AngularVelocity maxVelocity;
    private final IntakeWheelTelemetry telemetry;
    private final SparkMax motor;
    private final SlewRateLimiter profile;
    private final SimpleMotorFeedforward feedforward;
    private final MutAngularVelocity lastVelocity = RadiansPerSecond.mutable(0.0);
    private IntakeWheelRequest intakeWheelRequest;
    private FlywheelSim flywheelSim = null;
    private SparkMaxSim sparkMaxSim = null;

    public NEO550IntakeWheel(
            IntakeWheelConstants constants,
            SparkMax motor,
            Time updatePeriod) {
        this.motor = motor;
        this.maxVelocity = constants.maxVelocity();
        this.feedforward = new SimpleMotorFeedforward(constants.ks().baseUnitMagnitude(), constants.kv().baseUnitMagnitude(), constants.ka().baseUnitMagnitude(), updatePeriod.baseUnitMagnitude());
        this.profile = new SlewRateLimiter(constants.maxAcceleration().baseUnitMagnitude());
        this.telemetry = new IntakeWheelTelemetry(
                constants.name(),
                constants.maxVelocity(),
                constants.maxAcceleration());

        // if (RobotBase.isSimulation()) {
        //     DCMotor dcMotor = DCMotor.getNeo550(1);
        //     sparkMaxSim = new SparkMaxSim(motor, dcMotor);
        //     LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(
        //             constants.kv().baseUnitMagnitude(),
        //             constants.ka().baseUnitMagnitude());
        //     flywheelSim = new FlywheelSim(
        //             plant,
        //             dcMotor);
        // }
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return REV_SPARK_MAX;
    }

    @Override
    public IntakeWheelState getState() {
        return state;
    }

    @Override
    public AngularVelocity getMaxVelocity() {
        return maxVelocity;
    }

    @Override
    public void setControl(IntakeWheelRequest request) {
        if (intakeWheelRequest != request) {
            intakeWheelRequest = request;
        }
        request.apply(this);
    }

    @Override
    public void setVelocity(Dimensionless maxPercent) {
        double nextVelocitySetpoint = maxPercent.baseUnitMagnitude() * maxVelocity.baseUnitMagnitude();
        nextVelocitySetpoint = profile.calculate(nextVelocitySetpoint);
        double arbFeedforward = feedforward.calculateWithVelocities(lastVelocity.baseUnitMagnitude(), nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot1, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        lastVelocity.mut_setMagnitude(nextVelocitySetpoint);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        lastVelocity.mut_setMagnitude(motor.getEncoder().getVelocity());
        motor.getClosedLoopController().setReference(voltage.baseUnitMagnitude(), SparkBase.ControlType.kVoltage);
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.setAngularVelocity(motor.getEncoder().getVelocity());
        state.setVoltage(motor.getAppliedOutput() * motor.getBusVoltage());
        state.setTimestamp(Timer.getFPGATimestamp());
        state.setCurrent(motor.getOutputCurrent());
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