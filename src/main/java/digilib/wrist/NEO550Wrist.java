package digilib.wrist;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
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

public class NEO550Wrist extends Wrist {

    public enum ControlState {
        POSITION,
        VELOCITY
    }

    private final double minAngleRotations;
    private final double maxAngleRotations;
    private final double maxVelocityRPS;
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
            Config config,
            SparkMax motor,
            double controlPeriodSeconds,
            MechanismLigament2d top,
            MechanismLigament2d bottom) {
        super(
                config.name(),
                config.minAngleDegrees(),
                config.maxAngleDegrees(),
                config.maxVelocityRPS(),
                config.maxAccelerationRPSSquared());
        minAngleRotations = config.minAngleDegrees() / 360.0;
        maxAngleRotations = config.maxAngleDegrees() / 360.0;
        maxVelocityRPS = config.maxVelocityRPS();
        this.motor = motor;
        this.feedforward = new SimpleMotorFeedforward(
                config.ksVolts(),
                config.kvVoltsPerRPS(),
                config.kaVoltsPerRPSSquared(),
                controlPeriodSeconds);
        this.positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        config.maxControlVoltage(),
                        config.kvVoltsPerRPS(),
                        config.kaVoltsPerRPSSquared()));
        this.velocityProfile = new SlewRateLimiter(config.maxAccelerationRPSSquared());
        this.controlPeriodSeconds = controlPeriodSeconds;

        motor.getEncoder().setPosition(0.0);

        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeo550(1);
            sparkMaxSim = new SparkMaxSim(motor, dcMotor);
            LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(
                    config.kvVoltsPerRPS() / 2 / Math.PI,
                    config.kaVoltsPerRPSSquared() / 2 / Math.PI);
            simWrist = new DCMotorSim(
                    plant,
                    dcMotor);
            simWrist.setAngle(config.startingAngleDegrees() / 360.0);
            sparkMaxSim.setPosition(config.startingAngleDegrees() / 360.0);
            this.top = top;
            this.bottom = bottom;
        }
    }

    @Override
    public double getMotorEncoderPositionRotations() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getMotorEncoderPositionDegrees() {
        return motor.getEncoder().getPosition() * 360.0;
    }

    @Override
    public double getMotorEncoderVelocityDPS() {
        return motor.getEncoder().getVelocity() * 360.0;
    }

    @Override
    public void applyPositionRotations(double setpointRotations) {
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
    public void applyVelocity(double setpointScalar) {
        if (controlState != VELOCITY) {
            velocityProfile.reset(motor.getEncoder().getVelocity());
            setpoint.velocity = motor.getEncoder().getVelocity();
            controlState = VELOCITY;
        }
        double currentAngleRotations = motor.getEncoder().getPosition();
        if (currentAngleRotations >= maxAngleRotations && setpointScalar > 0.0) {
            applyPositionRotations(maxAngleRotations);
        } else if (currentAngleRotations <= minAngleRotations && setpointScalar < 0.0) {
            applyPositionRotations(minAngleRotations);
        } else {
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
    public double getVolts() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public double getAmps() {
        return motor.getOutputCurrent();
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getAppliedOutput() * supplyVoltage;
        simWrist.setInputVoltage(inputVoltage);
        simWrist.update(dt);
        sparkMaxSim.iterate(simWrist.getAngularVelocityRadPerSec(), supplyVoltage, dt);
        top.setLength(0.3 * Math.cos(Math.toRadians(getMotorEncoderPositionDegrees())));
        bottom.setLength(0.3 * Math.cos(Math.toRadians(getMotorEncoderPositionDegrees())));
    }
}
