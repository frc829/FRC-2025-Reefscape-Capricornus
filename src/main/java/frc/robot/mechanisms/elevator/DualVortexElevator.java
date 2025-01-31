package frc.robot.mechanisms.elevator;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;

public class DualVortexElevator extends Elevator {

    private final SparkFlex primaryMotor;
    private final SparkFlex followerMotor;
    private final SparkBaseConfig primaryMotorConfig;
    private final SparkBaseConfig followerMotorConfig;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ClosedLoopSlot positionClosedLoopSlot;
    private final ClosedLoopSlot velocityClosedLoopSlot;
    private final ExponentialProfile.State goalState = new ExponentialProfile.State();
    private final ElevatorFeedforward feedforward;
    private final Time profilePeriod;
    private final MutDistance position = Meters.mutable(0.0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private ExponentialProfile.State lastState = new ExponentialProfile.State();
    private ControlState controlState = ControlState.VELOCITY;

    public DualVortexElevator(
            ElevatorControlParameters elevatorControlParameters,
            SparkFlex primaryMotor,
            SparkFlex followerMotor,
            SparkBaseConfig primaryMotorConfig,
            SparkBaseConfig followerMotorConfig,
            ClosedLoopSlot positionClosedLoopSlot,
            ClosedLoopSlot velocityClosedLoopSlot) {
        super(elevatorControlParameters);
        this.primaryMotor = primaryMotor;
        this.followerMotor = followerMotor;
        this.primaryMotorConfig = primaryMotorConfig;
        this.followerMotorConfig = followerMotorConfig;
        this.positionClosedLoopSlot = positionClosedLoopSlot;
        this.velocityClosedLoopSlot = velocityClosedLoopSlot;
        this.feedforward = new ElevatorFeedforward(
                elevatorControlParameters.getKs().baseUnitMagnitude(),
                elevatorControlParameters.getKg().baseUnitMagnitude(),
                elevatorControlParameters.getKv().baseUnitMagnitude(),
                elevatorControlParameters.getKa().baseUnitMagnitude(),
                elevatorControlParameters.getUpdatePeriod().baseUnitMagnitude());
        this.positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        12.0,
                        elevatorControlParameters.getKv().baseUnitMagnitude(),
                        elevatorControlParameters.getKa().baseUnitMagnitude()));
        double maxAcceleration = feedforward.maxAchievableAcceleration(12.0, 0.0);
        this.velocityProfile = new SlewRateLimiter(maxAcceleration);
        this.profilePeriod = elevatorControlParameters.getUpdatePeriod();

    }

    @Override
    public boolean setNeutralModeToBrake() {
        primaryMotorConfig.idleMode(kBrake);
        followerMotorConfig.idleMode(kBrake);
        REVLibError primaryMotorConfigStatus = primaryMotor.configureAsync(primaryMotorConfig, kNoResetSafeParameters, kPersistParameters);
        REVLibError followerMotorConfigStatus = followerMotor.configureAsync(primaryMotorConfig, kNoResetSafeParameters, kPersistParameters);
        return primaryMotorConfigStatus == REVLibError.kOk && followerMotorConfigStatus == REVLibError.kOk;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        primaryMotorConfig.idleMode(kCoast);
        followerMotorConfig.idleMode(kCoast);
        REVLibError primaryMotorConfigStatus = primaryMotor.configureAsync(primaryMotorConfig, kNoResetSafeParameters, kPersistParameters);
        REVLibError followerMotorConfigStatus = followerMotor.configureAsync(primaryMotorConfig, kNoResetSafeParameters, kPersistParameters);
        return primaryMotorConfigStatus == REVLibError.kOk && followerMotorConfigStatus == REVLibError.kOk;
    }

    @Override
    public void setVelocity(LinearVelocity velocity) {
        goalState.velocity  = velocity.baseUnitMagnitude();
        controlState = ControlState.VELOCITY;
    }

    @Override
    public void setPosition(Distance position) {
        goalState.position = position.baseUnitMagnitude();
        goalState.velocity = 0.0;
        controlState = ControlState.POSITION;
    }

    @Override
    public void setHold() {
        if(controlState != ControlState.HOLD){
            goalState.position = primaryMotor.getEncoder().getPosition();
            goalState.velocity = 0.0;
            controlState = ControlState.HOLD;
        }
    }

    @Override
    public void update() {
        super.update();
        elevatorState.withPosition(position.mut_setMagnitude(primaryMotor.getEncoder().getPosition()));
        elevatorState.withVelocity(velocity.mut_setMagnitude(primaryMotor.getEncoder().getVelocity()));
        elevatorState.withTimestamp(timestamp.mut_setMagnitude(Timer.getFPGATimestamp()));
        switch (controlState) {
            case VELOCITY -> applyVelocity();
            case POSITION, HOLD -> applyPosition();
        }
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }

    @Override
    public void resetPosition() {
        primaryMotor.getEncoder().setPosition(0.0);
        followerMotor.getEncoder().setPosition(0.0);
    }

    private void applyVelocity() {
         double nextVelocitySetpoint = velocityProfile.calculate(goalState.velocity);
         double lastVelocitySetPoint = lastState.velocity;
         double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetPoint, nextVelocitySetpoint);
         primaryMotor.getClosedLoopController().setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, velocityClosedLoopSlot, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
         lastState.position = primaryMotor.getEncoder().getPosition();
         lastState.velocity = nextVelocitySetpoint;
    }

    private void applyPosition() {
        double lastVelocitySetpoint = lastState.velocity;
        lastState = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), lastState, goalState);
        double nextVelocitySetpoint = lastState.velocity;
        double nextPositionSetpoint = lastState.position;
        double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetpoint);
        primaryMotor.getClosedLoopController().setReference(nextPositionSetpoint, SparkBase.ControlType.kPosition, positionClosedLoopSlot, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        velocityProfile.reset(nextVelocitySetpoint);
    }
}
