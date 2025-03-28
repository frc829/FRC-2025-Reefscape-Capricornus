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

public class TwoVortexElevator implements Elevator {

    public enum ControlState {
        POSITION,
        VELOCITY,
        VOLTAGE
    }

    private final ElevatorState state = new ElevatorState();
    private final double minHeightMeters;
    private final double maxHeightMeters;
    private final double maxVelocityMPS;
    private final ElevatorTelemetry telemetry;
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
            ElevatorConstants constants,
            SparkFlex motor,
            SparkFlex follower,
            double controlPeriodSeconds,
            MechanismLigament2d ligament,
            double minimumHeight) {
        minHeightMeters = constants.minHeightMeters();
        maxHeightMeters = constants.maxHeightMeters();
        maxVelocityMPS = constants.maxVelocityMPS();
        this.motor = motor;
        telemetry = new ElevatorTelemetry(
                constants.name(),
                constants.minHeightMeters(),
                constants.maxHeightMeters(),
                constants.maxVelocityMPS(),
                constants.maxAccelerationMPSSquared());
        this.ligament = ligament;
        this.minimumHeight = minimumHeight;
        feedforward = new ElevatorFeedforward(
                constants.ksVolts(),
                constants.kgVolts(),
                constants.kvVoltsPerMPS(),
                constants.kaVoltsPerMPSSquared(),
                controlPeriodSeconds);
        positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        constants.maxControlVoltage(),
                        constants.kvVoltsPerMPS() / 0.90,
                        constants.kaVoltsPerMPSSquared() / 0.25));
        this.velocityProfile = new SlewRateLimiter(constants.maxAccelerationMPSSquared());
        this.controlPeriodSeconds = controlPeriodSeconds;

        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeoVortex(2);
            sparkFlexSim = new SparkFlexSim(motor, dcMotor);
            followerSparkFlexSim = new SparkFlexSim(follower, dcMotor);
            simElevator = SimulatedElevator.createFromSysId(
                    constants.ksVolts(),
                    constants.kgVolts(),
                    constants.kvVoltsPerMPS(),
                    constants.kaVoltsPerMPSSquared(),
                    dcMotor,
                    constants.startingHeightMeters(),
                    minHeightMeters,
                    maxHeightMeters);
            sparkFlexSim.setPosition(constants.startingHeightMeters());
            followerSparkFlexSim.setPosition(constants.startingHeightMeters());
        }
    }

    @Override
    public ElevatorState getState() {
        return state;
    }

    @Override
    public void setPosition(double setpointMeters) {
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
    public void setVelocity(double setpointScalar) {
        if (controlState != VELOCITY) {
            velocityProfile.reset(motor.getEncoder().getVelocity());
            setpoint.velocity = motor.getEncoder().getVelocity();
            controlState = VELOCITY;
        }
        double currentHeightMeters = motor.getEncoder().getPosition();
        if (currentHeightMeters >= maxHeightMeters && setpointScalar > 0.0){
            goal.velocity = 0.0;
        }else if(currentHeightMeters <= minHeightMeters && setpointScalar < 0.0){
            goal.velocity = 0.0;
        }else{
            goal.velocity = setpointScalar * maxVelocityMPS;
        }
        double nextVelocitySetpoint = velocityProfile.calculate(goal.velocity);
        double arbFeedfoward = feedforward.calculateWithVelocities(setpoint.velocity, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextVelocitySetpoint, kVelocity, kSlot1, arbFeedfoward, kVoltage);
        setpoint.velocity = nextVelocitySetpoint;

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
        state.setMotorEncoderPositionMeters(motor.getEncoder().getPosition());
        state.setMotorEncoderVelocityMPS(motor.getEncoder().getVelocity());
        state.setVolts(motor.getAppliedOutput() * motor.getBusVoltage());
        state.setAmps(motor.getOutputCurrent());
        state.setPositionSetpointMeters(setpoint.position);
        state.setVelocitySetpointMPS(setpoint.velocity);
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getAppliedOutput() * supplyVoltage;
        simElevator.setInputVoltage(inputVoltage);
        simElevator.update(dt);
        sparkFlexSim.iterate(simElevator.getVelocityMetersPerSecond(), supplyVoltage, dt);
        followerSparkFlexSim.iterate(simElevator.getVelocityMetersPerSecond(), supplyVoltage, dt);
        ligament.setLength(minimumHeight + state.getMotorEncoderPositionMeters());
    }
}
