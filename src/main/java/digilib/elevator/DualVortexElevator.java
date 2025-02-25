package digilib.elevator;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import digilib.MotorControllerType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static com.revrobotics.spark.ClosedLoopSlot.*;

public class DualVortexElevator implements Elevator {

    public enum ControlState{
        POSITION,
        VELOCITY,
        VOLTAGE
    }

    private final ElevatorState state = new ElevatorState();
    private final Distance minHeight;
    private final Distance maxHeight;
    private final LinearVelocity maxVelocity;
    private final ElevatorTelemetry telemetry;
    private ElevatorRequest elevatorRequest;
    private final SparkFlex motor;
    private final SparkFlex follower;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ExponentialProfile.State goalState = new ExponentialProfile.State();
    private ExponentialProfile.State lastState = new ExponentialProfile.State();
    private final ElevatorFeedforward feedforward;
    private final Time profilePeriod;
    private SimulatedElevator simElevator = null;
    private SparkFlexSim sparkFlexSim = null;
    private SparkFlexSim followerSparkFlexSim = null;
    private ControlState controlState = null;
    private final PIDController pidController;

    public DualVortexElevator(
            ElevatorConstants constants,
            SparkFlex motor,
            SparkFlex follower,
            Time updatePeriod) {
        minHeight = constants.minHeight();
        maxHeight = constants.maxHeight();
        maxVelocity = constants.maxVelocity();
        this.motor = motor;
        this.follower = follower;
        this.telemetry = new ElevatorTelemetry(
                constants.name(),
                constants.minHeight(),
                constants.maxHeight(),
                constants.maxVelocity(),
                constants.maxAcceleration());
        this.feedforward = new ElevatorFeedforward(
                constants.ks().baseUnitMagnitude(),
                constants.kg().baseUnitMagnitude(),
                constants.kv().baseUnitMagnitude(),
                constants.ka().baseUnitMagnitude(),
                updatePeriod.baseUnitMagnitude());
        this.positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        6.0,
                        constants.kv().baseUnitMagnitude(),
                        constants.ka().baseUnitMagnitude()));
        this.velocityProfile = new SlewRateLimiter(constants.maxAcceleration().baseUnitMagnitude());
        this.profilePeriod = updatePeriod;

        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeoVortex(2);
            sparkFlexSim = new SparkFlexSim(motor, dcMotor);
            followerSparkFlexSim = new SparkFlexSim(follower, dcMotor);
            LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(
                    constants.kv().baseUnitMagnitude(),
                    constants.ka().baseUnitMagnitude());
            simElevator = SimulatedElevator.createFromSysId(
                    constants.kg().baseUnitMagnitude(),
                    constants.kv().baseUnitMagnitude(),
                    constants.ka().baseUnitMagnitude(),
                    dcMotor,
                    constants.reduction(),
                    constants.startingHeight().baseUnitMagnitude(),
                    minHeight.baseUnitMagnitude(),
                    maxHeight.baseUnitMagnitude());
            sparkFlexSim.setPosition(constants.startingHeight().baseUnitMagnitude());
            followerSparkFlexSim.setPosition(constants.startingHeight().baseUnitMagnitude());
        }
        pidController = new PIDController(27.821, 0.0, 2.1438, 0.020);
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return MotorControllerType.REV_SPARK_FLEX;
    }

    @Override
    public Distance getMaxHeight() {
        return maxHeight;
    }

    @Override
    public Distance getMinHeight() {
        return minHeight;
    }

    @Override
    public LinearVelocity getMaxVelocity() {
        return maxVelocity;
    }

    @Override
    public ElevatorState getState() {
        return state;
    }

    @Override
    public void setControl(ElevatorRequest request) {
        if (elevatorRequest != request) {
            elevatorRequest = request;
        }
        request.apply(this);
    }

   @Override
   public void setHeight(Distance height) {
       if(controlState != ControlState.POSITION) {
           lastState.position = motor.getEncoder().getPosition();
           lastState.velocity = motor.getEncoder().getVelocity();
           controlState = ControlState.POSITION;
       }
       goalState.position = height.baseUnitMagnitude();
       goalState.velocity = 0.0;
       double lastVelocitySetpoint = lastState.velocity;
       lastState = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), lastState, goalState);
       double nextVelocitySetpoint = lastState.velocity;
       double nextPositionSetpoint = lastState.position;
       double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetpoint);
       motor.getClosedLoopController().setReference(nextPositionSetpoint, SparkBase.ControlType.kPosition, kSlot0, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
   }

    // @Override
    // public void setHeight(Distance height) {
    //     if(controlState != ControlState.POSITION) {
    //         lastState.position = motor.getEncoder().getPosition();
    //         lastState.velocity = motor.getEncoder().getVelocity();
    //         controlState = ControlState.POSITION;
    //     }
    //     goalState.position = height.baseUnitMagnitude();
    //     goalState.velocity = 0.0;
    //     double lastVelocitySetpoint = lastState.velocity;
    //     lastState = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), lastState, goalState);
    //     double nextVelocitySetpoint = lastState.velocity;
    //     double nextPositionSetpoint = lastState.position;
    //     double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetpoint);
    //     double feedback = pidController.calculate(nextPositionSetpoint, goalState.position);
    //     double voltage = MathUtil.clamp(arbFeedfoward + feedback, -6.0, 6.0);
    //     motor.setVoltage(voltage);
    // }

    @Override
    public void setVelocity(Dimensionless maxPercent) {
        if(controlState != ControlState.VELOCITY) {
            velocityProfile.reset(motor.getEncoder().getVelocity());
            controlState = ControlState.VELOCITY;
        }
        double goalVelocity = maxPercent.baseUnitMagnitude() * maxVelocity.baseUnitMagnitude();
        double nextVelocitySetpoint = velocityProfile.calculate(goalVelocity);
        double lastVelocitySetPoint = motor.getEncoder().getVelocity();
        double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetPoint, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, kSlot1, arbFeedfoward);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        if(controlState != ControlState.VOLTAGE) {
            controlState = ControlState.VOLTAGE;
        }
        motor.setVoltage(voltage.baseUnitMagnitude());
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(0.0);
        follower.getEncoder().setPosition(0.0);
        updateState();
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.setHeight(motor.getEncoder().getPosition());
        state.setVelocity(motor.getEncoder().getVelocity());
        state.setVoltage(motor.getAppliedOutput() * motor.getBusVoltage());
        state.setTimestamp(Timer.getFPGATimestamp());
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getAppliedOutput() * 12.0;
        simElevator.setInputVoltage(inputVoltage);
        simElevator.update(dt);
        sparkFlexSim.iterate(simElevator.getVelocityMetersPerSecond(), 12.0, dt);
        followerSparkFlexSim.iterate(simElevator.getVelocityMetersPerSecond(), 12.0, dt);
    }
}
