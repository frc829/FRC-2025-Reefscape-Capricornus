package digilib.wrist;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.*;
import digilib.MotorControllerType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.ClosedLoopSlot.kSlot1;
import static edu.wpi.first.units.Units.*;

public class NEO550Wrist implements Wrist {
    private final WristState state = new WristState();
    private final Angle minAngle;
    private final Angle maxAngle;
    private final AngularVelocity maxVelocity;
    private final WristTelemetry telemetry;
    private WristRequest wristRequest;
    private final SparkMax motor;
    private final CANcoder cancoder;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ExponentialProfile.State goalState = new ExponentialProfile.State();
    private final SimpleMotorFeedforward feedforward;
    private final Time profilePeriod;
    private ExponentialProfile.State lastState = new ExponentialProfile.State();
    private boolean hold = false;
    private DCMotorSim simWrist = null;
    private SparkMaxSim sparkMaxSim = null;
    private CANcoderSimState canCoderSimState = null;

    public NEO550Wrist(
            WristConstants constants,
            SparkMax motor,
            CANcoder cancoder,
            Time updatePeriod) {
        minAngle = constants.minAngle();
        maxAngle = constants.maxAngle();
        maxVelocity = constants.maxAngularVelocity();
        this.motor = motor;
        this.cancoder = cancoder;
        this.telemetry = new WristTelemetry(
                "Wrist",
                constants.minAngle(),
                constants.maxAngle(),
                constants.maxAngularVelocity(),
                constants.maxAngularAcceleration());
        this.feedforward = new SimpleMotorFeedforward(
                constants.ks().baseUnitMagnitude(),
                constants.kv().baseUnitMagnitude(),
                constants.ka().baseUnitMagnitude(),
                updatePeriod.baseUnitMagnitude());
        this.positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        12.0,
                        constants.kv().baseUnitMagnitude(),
                        constants.ka().baseUnitMagnitude()));
        this.velocityProfile = new SlewRateLimiter(constants.maxAngularAcceleration().baseUnitMagnitude());
        this.profilePeriod = updatePeriod;

        double absolutePositionRotations = cancoder.getAbsolutePosition().getValueAsDouble();
        absolutePositionRotations = MathUtil.inputModulus(absolutePositionRotations, -0.5, 0.5);
        double absolutePositionRadians = absolutePositionRotations * 2 * Math.PI;
        motor.getEncoder().setPosition(absolutePositionRadians);
        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeo550(1);
            sparkMaxSim = new SparkMaxSim(motor, dcMotor);
            canCoderSimState = cancoder.getSimState();
            LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(
                    constants.kv().baseUnitMagnitude(),
                    constants.ka().baseUnitMagnitude());
            simWrist = new DCMotorSim(
                    plant,
                    dcMotor,
                    constants.positionStdDev().baseUnitMagnitude(),
                    constants.velocityStdDev().baseUnitMagnitude());
            simWrist.setState(absolutePositionRadians, 0.0);
            sparkMaxSim.setPosition(absolutePositionRadians);
        }
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return MotorControllerType.REV_SPARK_MAX;
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
        return maxVelocity;
    }

    @Override
    public WristState getState() {
        return state;
    }

    @Override
    public void setControl(WristRequest request) {
        if (wristRequest != request) {
            wristRequest = request;
        }
        request.apply(this);
    }

    @Override
    public void setPosition(Angle position) {
        goalState.position = position.baseUnitMagnitude();
        goalState.velocity = 0.0;
        double lastVelocitySetpoint = lastState.velocity;
        lastState = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), lastState, goalState);
        double nextVelocitySetpoint = lastState.velocity;
        double nextPositionSetpoint = lastState.position;
        double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextPositionSetpoint, SparkBase.ControlType.kPosition, kSlot0, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        velocityProfile.reset(motor.getEncoder().getVelocity());
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        goalState.velocity = velocity.baseUnitMagnitude();
        double nextVelocitySetpoint = velocityProfile.calculate(goalState.velocity);
        double lastVelocitySetPoint = lastState.velocity;
        double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetPoint, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, kSlot1, arbFeedfoward);
        lastState.position = motor.getEncoder().getPosition();
        lastState.velocity = nextVelocitySetpoint;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.baseUnitMagnitude());
        velocityProfile.reset(motor.getEncoder().getVelocity());
        lastState.position = motor.getEncoder().getPosition();
        lastState.velocity = motor.getEncoder().getVelocity();
    }

    @Override
    public void resetPosition() {
    }

    @Override
    public void update() {
        double absolutePositionRotations = cancoder.getAbsolutePosition().getValueAsDouble();
        absolutePositionRotations = MathUtil.inputModulus(absolutePositionRotations, -0.5, 0.5);
        double absolutePositionRadians = absolutePositionRotations * 2 * Math.PI;
        motor.getEncoder().setPosition(absolutePositionRadians);
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.withPosition(motor.getEncoder().getPosition())
                .withAbsolutePosition(cancoder.getAbsolutePosition().getValue().in(Radians))
                .withVelocity(motor.getEncoder().getVelocity())
                .withAbsoluteVelocity(cancoder.getVelocity().getValue().in(RadiansPerSecond))
                .withVoltage(motor.getAppliedOutput() * motor.getBusVoltage())
                .withTimestamp(Timer.getFPGATimestamp())
                .withAbsoluteEncoderStatus(cancoder.getMagnetHealth().getValue().name());
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getAppliedOutput() * 12.0;
        simWrist.setInputVoltage(inputVoltage);
        simWrist.update(dt);

        double rotations = simWrist.getAngularPositionRotations();
        rotations = MathUtil.inputModulus(rotations, -0.5, 0.5);

        canCoderSimState.setSupplyVoltage(supplyVoltage);
        canCoderSimState.setMagnetHealth(MagnetHealthValue.Magnet_Green);
        canCoderSimState.setVelocity(simWrist.getAngularVelocityRPM() / 60.0);
        canCoderSimState.setRawPosition(rotations);

        sparkMaxSim.iterate(simWrist.getAngularVelocityRadPerSec(), 12.0, dt);
    }
}
