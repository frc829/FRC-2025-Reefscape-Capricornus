package digilib.elevator;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.ClosedLoopSlot.kSlot1;
import static com.revrobotics.spark.SparkBase.ControlType.kPosition;
import static com.revrobotics.spark.SparkBase.ControlType.kVelocity;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;
import static digilib.elevator.TwoVortexElevator.ControlState.*;

public class TwoVortexElevator extends Elevator {

    public enum ControlState {
        POSITION,
        VELOCITY,
        VOLTAGE
    }

    private final double minHeightMeters;
    private final double maxHeightMeters;
    private final double maxVelocityMPS;
    private final MechanismLigament2d ligament;
    private final double minimumHeight;
    private final SparkFlex motor;
    private ControlState controlState = null;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ExponentialProfile.State goal = new ExponentialProfile.State();
    private ExponentialProfile.State setpoint = new ExponentialProfile.State();
    private final ElevatorFeedforward feedforward;
    private final double controlPeriodSeconds;
    private SimulatedElevator simElevator = null;
    private SparkFlexSim sparkFlexSim = null;
    private SparkFlexSim followerSparkFlexSim = null;

    public TwoVortexElevator(
            Config config,
            SparkFlex motor,
            SparkFlex follower,
            double controlPeriodSeconds,
            MechanismLigament2d ligament,
            double minimumHeight) {
        super(
                config.name(),
                config.minHeightMeters(),
                config.maxHeightMeters(),
                config.maxVelocityMPS(),
                config.maxAccelerationMPSSquared());
        minHeightMeters = config.minHeightMeters();
        maxHeightMeters = config.maxHeightMeters();
        maxVelocityMPS = config.maxVelocityMPS();
        this.motor = motor;
        this.ligament = ligament;
        this.minimumHeight = minimumHeight;
        feedforward = new ElevatorFeedforward(
                config.ksVolts(),
                config.kgVolts(),
                config.kvVoltsPerMPS(),
                config.kaVoltsPerMPSSquared(),
                controlPeriodSeconds);
        positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        config.maxControlVoltage(),
                        config.kvVoltsPerMPS() / 0.90,
                        config.kaVoltsPerMPSSquared() / 0.25));
        this.velocityProfile = new SlewRateLimiter(config.maxAccelerationMPSSquared());
        this.controlPeriodSeconds = controlPeriodSeconds;

        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeoVortex(2);
            sparkFlexSim = new SparkFlexSim(motor, dcMotor);
            followerSparkFlexSim = new SparkFlexSim(follower, dcMotor);
            simElevator = SimulatedElevator.createFromSysId(
                    config.ksVolts(),
                    config.kgVolts(),
                    config.kvVoltsPerMPS(),
                    config.kaVoltsPerMPSSquared(),
                    dcMotor,
                    config.startingHeightMeters(),
                    minHeightMeters,
                    maxHeightMeters);
            sparkFlexSim.setPosition(config.startingHeightMeters());
            followerSparkFlexSim.setPosition(config.startingHeightMeters());
        }
    }

    @Override
    public double getMotorEncoderPositionMeters() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getMotorEncoderVelocityMPS() {
        return motor.getEncoder().getVelocity();
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
    public void applyPosition(double setpointMeters) {
        if (controlState != POSITION) {
            setpoint.position = motor.getEncoder().getPosition();
            setpoint.velocity = motor.getEncoder().getVelocity();
            controlState = POSITION;
        }
        goal.velocity = 0.0;
        goal.position = setpointMeters;
        ExponentialProfile.State next = positionProfile
                .calculate(controlPeriodSeconds, setpoint, goal);
        double arbFeedfoward = feedforward
                .calculateWithVelocities(setpoint.velocity, next.velocity);
        setpoint = next;
        motor.getClosedLoopController()
                .setReference(next.position,
                        kPosition,
                        kSlot0,
                        arbFeedfoward,
                        kVoltage);

    }

    @Override
    public void applyVelocity(double setpointScalar) {
        if (controlState != VELOCITY) {
            velocityProfile.reset(motor.getEncoder().getVelocity());
            setpoint.velocity = motor.getEncoder().getVelocity();
            controlState = VELOCITY;
        }
        double currentHeightMeters = motor.getEncoder().getPosition();
        if (currentHeightMeters >= maxHeightMeters && setpointScalar > 0.0) {
            goal.velocity = 0.0;
        } else if (currentHeightMeters <= minHeightMeters && setpointScalar < 0.0) {
            goal.velocity = 0.0;
        } else {
            goal.velocity = setpointScalar * maxVelocityMPS;
        }
        double nextVelocitySetpoint = velocityProfile.calculate(goal.velocity);
        double arbFeedfoward = feedforward.calculateWithVelocities(setpoint.velocity, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextVelocitySetpoint, kVelocity, kSlot1, arbFeedfoward, kVoltage);
        setpoint.velocity = nextVelocitySetpoint;

    }

    @Override
    public void applyVoltage(double volts) {
        if (controlState != VOLTAGE) {
            controlState = VOLTAGE;
        }
        motor.setVoltage(volts);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getAppliedOutput() * supplyVoltage;
        simElevator.setInputVoltage(inputVoltage);
        simElevator.update(dt);
        sparkFlexSim.iterate(simElevator.getVelocityMetersPerSecond(), supplyVoltage, dt);
        followerSparkFlexSim.iterate(simElevator.getVelocityMetersPerSecond(), supplyVoltage, dt);
        ligament.setLength(minimumHeight + getMotorEncoderPositionMeters());
    }
}
