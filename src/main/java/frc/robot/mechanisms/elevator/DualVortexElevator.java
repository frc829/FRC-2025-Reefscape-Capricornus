package frc.robot.mechanisms.elevator;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.revrobotics.spark.ClosedLoopSlot.*;
import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;

public class DualVortexElevator implements Elevator {
    private final ElevatorState lastElevatorState = new ElevatorState();
    private final ElevatorState elevatorState = new ElevatorState();
    private final Distance minHeight;
    private final Distance maxHeight;
    private ElevatorRequest elevatorRequest;
    private final SparkFlex primaryMotor;
    private final SparkFlex followerMotor;
    private final SparkBaseConfig primaryMotorConfig;
    private final SparkBaseConfig followerMotorConfig;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ExponentialProfile.State goalState = new ExponentialProfile.State();
    private final ElevatorFeedforward feedforward;
    private final Time profilePeriod;
    private final MutDistance position = Meters.mutable(0.0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private ExponentialProfile.State lastState = new ExponentialProfile.State();
    private final ElevatorSim simElevator;
    private final SparkFlexSim primarySparkFlexSim;
    private final SparkFlexSim followerSparkFlexSim;
    private boolean hold = false;

    public DualVortexElevator(
            ElevatorConstants elevatorConstants,
            SparkFlex primaryMotor,
            SparkFlex followerMotor,
            SparkBaseConfig primaryMotorConfig,
            SparkBaseConfig followerMotorConfig,
            Time updatePeriod) {
        minHeight = elevatorConstants.getMinHeight();
        maxHeight = elevatorConstants.getMaxHeight();
        this.primaryMotor = primaryMotor;
        this.followerMotor = followerMotor;
        this.primaryMotorConfig = primaryMotorConfig;
        this.followerMotorConfig = followerMotorConfig;
        this.feedforward = new ElevatorFeedforward(
                elevatorConstants.getKs().baseUnitMagnitude(),
                elevatorConstants.getKg().baseUnitMagnitude(),
                elevatorConstants.getKv().baseUnitMagnitude(),
                elevatorConstants.getKa().baseUnitMagnitude(),
                updatePeriod.baseUnitMagnitude());
        this.positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        12.0,
                        elevatorConstants.getKv().baseUnitMagnitude(),
                        elevatorConstants.getKa().baseUnitMagnitude()));
        double maxAcceleration = feedforward.maxAchievableAcceleration(12.0, 0.0);
        this.velocityProfile = new SlewRateLimiter(maxAcceleration);
        this.profilePeriod = updatePeriod;
        DCMotor dcMotor = DCMotor.getNeoVortex(2);
        LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(
                elevatorConstants.getKv().baseUnitMagnitude(),
                elevatorConstants.getKa().baseUnitMagnitude());
        this.primarySparkFlexSim = new SparkFlexSim(primaryMotor, dcMotor);
        this.followerSparkFlexSim = new SparkFlexSim(followerMotor, dcMotor);
        this.simElevator = new ElevatorSim(
                plant,
                dcMotor,
                minHeight.baseUnitMagnitude(),
                maxHeight.baseUnitMagnitude(),
                true,
                Meters.of(0.0).baseUnitMagnitude());

    }

    @Override
    public boolean setNeutralModeToBrake() {
        primaryMotorConfig.idleMode(kBrake);
        followerMotorConfig.idleMode(kBrake);
        REVLibError primaryMotorConfigStatus = primaryMotor.configureAsync(primaryMotorConfig, kNoResetSafeParameters, kPersistParameters);
        REVLibError followerMotorConfigStatus = followerMotor.configureAsync(followerMotorConfig, kNoResetSafeParameters, kPersistParameters);
        return primaryMotorConfigStatus == REVLibError.kOk && followerMotorConfigStatus == REVLibError.kOk;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        primaryMotorConfig.idleMode(kCoast);
        followerMotorConfig.idleMode(kCoast);
        REVLibError primaryMotorConfigStatus = primaryMotor.configureAsync(primaryMotorConfig, kNoResetSafeParameters, kPersistParameters);
        REVLibError followerMotorConfigStatus = followerMotor.configureAsync(followerMotorConfig, kNoResetSafeParameters, kPersistParameters);
        return primaryMotorConfigStatus == REVLibError.kOk && followerMotorConfigStatus == REVLibError.kOk;
    }

    @Override
    public ElevatorState getState() {
        return elevatorState;
    }

    @Override
    public ElevatorState getStateCopy() {
        return elevatorState.clone();
    }

    @Override
    public ElevatorState getLastArmState() {
        return lastElevatorState;
    }

    @Override
    public void setControl(ElevatorRequest request) {
        if (elevatorRequest != request) {
            elevatorRequest = request;
        }
        request.apply(this);
    }

    @Override
    public void setPosition(Distance position) {
        goalState.position = position.baseUnitMagnitude();
        goalState.velocity = 0.0;
        double lastVelocitySetpoint = lastState.velocity;
        lastState = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), lastState, goalState);
        double nextVelocitySetpoint = lastState.velocity;
        double nextPositionSetpoint = lastState.position;
        double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetpoint);
        primaryMotor.getClosedLoopController().setReference(nextPositionSetpoint, SparkBase.ControlType.kPosition, kSlot0, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        velocityProfile.reset(nextVelocitySetpoint);
    }

    @Override
    public void setVelocity(LinearVelocity velocity) {
        goalState.velocity = velocity.baseUnitMagnitude();
        double nextVelocitySetpoint = velocityProfile.calculate(goalState.velocity);
        double lastVelocitySetPoint = lastState.velocity;
        double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetPoint, nextVelocitySetpoint);
        primaryMotor.getClosedLoopController().setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, kSlot1, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        lastState.position = primaryMotor.getEncoder().getPosition();
        lastState.velocity = nextVelocitySetpoint;
    }

    @Override
    public void resetPosition() {
        primaryMotor.getEncoder().setPosition(0.0);
        followerMotor.getEncoder().setPosition(0.0);
        updateState();
    }

    @Override
    public void update() {
        lastElevatorState.withElevatorState(elevatorState);
        updateState();
        updateTelemetry();
    }

    private void updateState() {
        elevatorState.withPosition(position.mut_setMagnitude(primaryMotor.getEncoder().getPosition()));
        elevatorState.withVelocity(velocity.mut_setMagnitude(primaryMotor.getEncoder().getVelocity()));
        elevatorState.withTimestamp(timestamp.mut_setMagnitude(Timer.getFPGATimestamp()));
    }

    @Override
    public void updateTelemetry() {
        SmartDashboard.putNumberArray("State [meters, mps]", new double[]{
                elevatorState.getPosition().in(Meters),
                elevatorState.getVelocity().in(MetersPerSecond)});
        SmartDashboard.putBoolean("Hold", isHoldEnabled());
        // TODO: will do later
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = primaryMotor.getAppliedOutput() * 12.0;
        simElevator.setInputVoltage(inputVoltage);
        simElevator.update(dt);
        primarySparkFlexSim.iterate(simElevator.getVelocityMetersPerSecond(), 12.0, dt);
        followerSparkFlexSim.iterate(simElevator.getVelocityMetersPerSecond(), 12.0, dt);
    }

    @Override
    public Distance getMaxPosition() {
        return maxHeight;
    }

    @Override
    public Distance getMinPosition() {
        return minHeight;
    }

    @Override
    public void disableHold() {
        hold = false;
    }

    @Override
    public void enableHold() {
        hold = true;
    }

    @Override
    public boolean isHoldEnabled() {
        return hold;
    }
}
