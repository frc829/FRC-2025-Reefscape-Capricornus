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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static edu.wpi.first.units.Units.*;

public class NEO550Wrist implements Wrist {

    public enum ControlState {
        POSITION,
        VELOCITY,
        VOLTAGE
    }

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
    private final ExponentialProfile.State goal = new ExponentialProfile.State();
    private final SimpleMotorFeedforward feedforward;
    private final Time profilePeriod;
    private ExponentialProfile.State setpoint = new ExponentialProfile.State();
    private DCMotorSim simWrist = null;
    private SparkMaxSim sparkMaxSim = null;
    private CANcoderSimState canCoderSimState = null;
    private ControlState controlState = null;

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
                constants.name(),
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
        // if (controlState != ControlState.POSITION) {
        //     setpoint.position = motor.getEncoder().getPosition();
        //     setpoint.velocity = motor.getEncoder().getVelocity();
        //     controlState = ControlState.POSITION;
        // }
        // goal.position = position.baseUnitMagnitude();
        // goal.velocity = 0.0;
        // ExponentialProfile.State next = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), setpoint, goal);
        // double currentVelocity = motor.getEncoder().getVelocity();
        // double arbFeedfoward = feedforward.calculateWithVelocities(currentVelocity, next.velocity);
        // motor.getClosedLoopController().setReference(next.position, SparkBase.ControlType.kPosition, kSlot0, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        // setpoint = next;
        motor.getClosedLoopController().setReference(position.baseUnitMagnitude(), SparkBase.ControlType.kPosition, kSlot0);
    }

    @Override
    public void setVelocity(Dimensionless maxPercent) {
        if (controlState != ControlState.VELOCITY) {
            velocityProfile.reset(motor.getEncoder().getVelocity());
            controlState = ControlState.VELOCITY;
        }
        motor.setVoltage(maxPercent.baseUnitMagnitude());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        if (controlState != ControlState.VOLTAGE) {
            controlState = ControlState.VOLTAGE;
        }
        motor.setVoltage(voltage.baseUnitMagnitude());
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
        state.setPosition(motor.getEncoder().getPosition());
        state.setSetpoint(setpoint.position);
        state.setAbsolutePosition(cancoder.getAbsolutePosition().getValue().in(Radians));
        state.setVelocity(motor.getEncoder().getVelocity());
        state.setAbsoluteVelocity(cancoder.getVelocity().getValue().in(RadiansPerSecond));
        state.setVoltage(motor.getAppliedOutput() * motor.getBusVoltage());
        state.setAbsoluteEncoderStatus(cancoder.getMagnetHealth().getValue().name());
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

// private final PIDController pidController;
// pidController = new PIDController(89.778, 0.0, 6.7465, 0.020);

//    @Override
//    public void setPosition(Angle position) {
//        if (controlState != ControlState.POSITION) {
//            setpoint.position = motor.getEncoder().getPosition();
//            setpoint.velocity = motor.getEncoder().getVelocity();
//            controlState = ControlState.POSITION;
//        }
//        goal.position = position.baseUnitMagnitude();
//        goal.velocity = 0.0;
//        ExponentialProfile.State next = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), setpoint, goal);
//        double currentPosition = motor.getEncoder().getPosition();
//        double currentVelocity = motor.getEncoder().getVelocity();
//        double arbFeedfoward = feedforward.calculateWithVelocities(currentVelocity, next.velocity);
//        double feedback = pidController.calculate(currentPosition, setpoint.position);
//        double voltage = arbFeedfoward + feedback;
//        if(RobotBase.isSimulation()) {
//            voltage = MathUtil.clamp(voltage, -12.0, 12.0);
//        }
//        motor.setVoltage(voltage);
//        setpoint = next;
//    }
