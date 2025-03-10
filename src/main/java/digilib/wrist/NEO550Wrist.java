package digilib.wrist;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import digilib.MotorControllerType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.ClosedLoopSlot.kSlot1;
import static com.revrobotics.spark.SparkBase.ControlType.kPosition;
import static com.revrobotics.spark.SparkBase.ControlType.kVelocity;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;
import static digilib.wrist.NEO550Wrist.ControlState.POSITION;
import static digilib.wrist.NEO550Wrist.ControlState.VELOCITY;

public class NEO550Wrist implements Wrist {

    public enum ControlState {
        POSITION,
        VELOCITY,
        VOLTAGE
    }

    private final WristState state = new WristState();
    private final double minAngleRotations;
    private final double maxAngleRotations;
    private final double maxVelocityRPS;
    private final WristTelemetry telemetry;
    private final SparkMax motor;
    private ControlState controlState = null;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ExponentialProfile.State goal = new ExponentialProfile.State();
    private ExponentialProfile.State setpoint = new ExponentialProfile.State();
    private final SimpleMotorFeedforward feedforward;
    private final double controlPeriodSeconds;
    private DCMotorSim simWrist = null;
    private SparkMaxSim sparkMaxSim = null;
    private MechanismLigament2d top = null;
    private MechanismLigament2d bottom = null;

    public NEO550Wrist(
            WristConstants constants,
            SparkMax motor,
            double controlPeriodSeconds,
            MechanismLigament2d top,
            MechanismLigament2d bottom) {
        minAngleRotations = constants.minAngleDegrees() / 360.0;
        maxAngleRotations = constants.maxAngleDegrees() / 360.0;
        maxVelocityRPS = constants.maxVelocityRPS();
        this.motor = motor;
        this.telemetry = new WristTelemetry(
                constants.name(),
                constants.minAngleDegrees(),
                constants.maxAngleDegrees(),
                constants.maxVelocityRPS(),
                constants.maxAccelerationRPSSquared());
        this.feedforward = new SimpleMotorFeedforward(
                constants.ksVolts(),
                constants.kvVoltsPerRPS(),
                constants.kaVoltsPerRPSSquared(),
                controlPeriodSeconds);
        this.positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        constants.maxControlVoltage(),
                        constants.kvVoltsPerRPS(),
                        constants.kaVoltsPerRPSSquared()));
        this.velocityProfile = new SlewRateLimiter(constants.maxAccelerationRPSSquared());
        this.controlPeriodSeconds = controlPeriodSeconds;

        motor.getEncoder().setPosition(0.0);

        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeo550(1);
            sparkMaxSim = new SparkMaxSim(motor, dcMotor);
            LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(
                    constants.kvVoltsPerRPS() / 2 / Math.PI,
                    constants.kaVoltsPerRPSSquared() / 2 / Math.PI);
            simWrist = new DCMotorSim(
                    plant,
                    dcMotor);
            simWrist.setAngle(constants.startingAngleDegrees() / 360.0);
            sparkMaxSim.setPosition(constants.startingAngleDegrees() / 360.0);
            this.top = top;
            this.bottom = bottom;
        }
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return MotorControllerType.REV_SPARK_MAX;
    }

    @Override
    public double getMinAngleRotations() {
        return minAngleRotations;
    }

    @Override
    public double getMaxAngleRotations() {
        return maxAngleRotations;
    }

    @Override
    public double getMaxVelocityRPS() {
        return maxVelocityRPS;
    }

    @Override
    public WristState getState() {
        return state;
    }

    @Override
    public void setPosition(double setpointRotations) {
        if (controlState != POSITION) {
            setpoint.position = motor.getEncoder().getPosition();
            setpoint.velocity = motor.getEncoder().getVelocity();
            controlState = POSITION;
        }
        double currentAngleRotations = motor.getEncoder().getPosition();
        goal.velocity = 0.0;
        if (currentAngleRotations >= maxAngleRotations && setpointRotations > maxVelocityRPS) {
            goal.position = maxAngleRotations;
        } else if (currentAngleRotations <= minAngleRotations && setpointRotations < minAngleRotations) {
            goal.position = minAngleRotations;
        } else {
            goal.position = setpointRotations;
        }
        ExponentialProfile.State next = positionProfile
                .calculate(controlPeriodSeconds, setpoint, goal);
        double feedforwardVolts = feedforward
                .calculateWithVelocities(setpoint.velocity, next.velocity);
        motor.getClosedLoopController()
                .setReference(next.position,
                        kPosition,
                        kSlot0,
                        feedforwardVolts,
                        kVoltage);
        setpoint = next;
    }

    @Override
    public void setVelocity(double setpointScalar) {
        if (controlState != VELOCITY) {
            velocityProfile.reset(motor.getEncoder().getVelocity());
            setpoint.velocity = motor.getEncoder().getVelocity();
            controlState = VELOCITY;
        }
        double currentAngleRotations = motor.getEncoder().getPosition();
        if (currentAngleRotations >= maxAngleRotations && setpointScalar > 0.0){
            setPosition(maxAngleRotations);
        }else if(currentAngleRotations <= minAngleRotations && setpointScalar < 0.0){
            setPosition(minAngleRotations);
        }else{
            goal.velocity = setpointScalar * maxVelocityRPS;
            double nextVelocitySetpoint = velocityProfile.calculate(goal.velocity);
            double feedforwardVolts = feedforward
                    .calculateWithVelocities(setpoint.velocity, nextVelocitySetpoint);
            motor.getClosedLoopController()
                    .setReference(nextVelocitySetpoint,
                            kVelocity,
                            kSlot1,
                            feedforwardVolts,
                            kVoltage);
            setpoint.velocity = nextVelocitySetpoint;
        }
    }

    @Override
    public void setVoltage(double volts) {
        if (controlState != ControlState.VOLTAGE) {
            controlState = ControlState.VOLTAGE;
        }
        motor.setVoltage(volts);
    }

    @Override
    public void resetPosition() {
    }

    @Override
    public void update() {
        // resetPosition();
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.setMotorEncoderPositionRotations(motor.getEncoder().getPosition());
        state.setMotorEncoderVelocityRPS(motor.getEncoder().getVelocity());
        state.setVolts(motor.getAppliedOutput() * motor.getBusVoltage());
        state.setAmps(motor.getOutputCurrent());
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
        sparkMaxSim.iterate(simWrist.getAngularVelocityRadPerSec(), 12.0, dt);
        top.setLength(0.3 * Math.cos(Math.toRadians(state.getMotorEncoderPositionDegrees())));
        bottom.setLength(0.3 * Math.cos(Math.toRadians(state.getMotorEncoderPositionDegrees())));
    }
}
