package frc.robot.mechanisms.wrist;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;

public class NEO550Wrist extends Wrist {

    public enum ControlState {
        POSITION,
        VELOCITY,
        HOLD,
    }

    private final SparkFlex primaryMotor;
    private final SparkBaseConfig primaryMotorConfig;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ClosedLoopSlot positionClosedLoopSlot;
    private final ClosedLoopSlot velocityClosedLoopSlot;
    private final ExponentialProfile.State goalState = new ExponentialProfile.State();
    private final SimpleMotorFeedforward feedforward;
    private final Time profilePeriod;
    private final MutDistance position = Meters.mutable(0.0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private ExponentialProfile.State lastState = new ExponentialProfile.State();
    private ControlState controlState = ControlState.VELOCITY;

    public NEO550Wrist(
            WristControlParameters wristControlParameters,
            SparkFlex primaryMotor,
            SparkBaseConfig primaryMotorConfig,
            ClosedLoopSlot positionClosedLoopSlot,
            ClosedLoopSlot velocityClosedLoopSlot) {
        super(wristControlParameters);
        this.primaryMotor = primaryMotor;
        this.primaryMotorConfig = primaryMotorConfig;
        this.positionClosedLoopSlot = positionClosedLoopSlot;
        this.velocityClosedLoopSlot = velocityClosedLoopSlot;
        this.feedforward = new SimpleMotorFeedforward(
                wristControlParameters.getKs().baseUnitMagnitude(),
                wristControlParameters.getKv().baseUnitMagnitude(),
                wristControlParameters.getKa().baseUnitMagnitude(),
                wristControlParameters.getUpdatePeriod().baseUnitMagnitude());
        this.positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        12.0,
                        wristControlParameters.getKv().baseUnitMagnitude(),
                        wristControlParameters.getKa().baseUnitMagnitude()));
        double maxAcceleration = feedforward.maxAchievableAcceleration(12.0, 0.0);
        this.velocityProfile = new SlewRateLimiter(maxAcceleration);
        this.profilePeriod = wristControlParameters.getUpdatePeriod();

    }

    @Override
    public boolean setNeutralModeToBrake() {
        // TODO: call primaryMotorConfig's idleMode method and pass in kBrake
        // TODO: create a REVLibError variable called primaryMotorConfigStatus assign primaryMotor.configureAsync passing in primaryMotorConfig, kNoResetSafeParameters, and  kPersistParameters
        // TODO: return primaryMotorConfigStatus == REVLibError.kOk && followerMotorConfigStatus == REVLibError.kOk;
        return false;  // TODO: remove this when done.  Since you've returned in the previous line.
    }

    @Override
    public boolean setNeutralModeToCoast() {
        // TODO: identical to setNeutralModeToBrake but with kCoast instead of kBrake
        return false;  // TODO: remove this when done.  Since you've returned in the previous line.
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        // TODO: assign velocity.baseUnitMagnitude() to goalState.velocity
        // TODO: assign ControlState.VELOCITY to controlState
    }

    @Override
    public void setPosition(Angle position) {
        // TODO: assign position.baseUnitMagnitude() to goalState.position
        // TODO: assign 0.0 to goalState.velocity
        // TODO: assign ControlState.POSITION to controlState
    }

    @Override
    public void setHold() {
        // TODO: if the controlState is not equal to HOLD
        // TODO: then do the following
        // TODO: assign primaryMotor.getEncoder().getPosition() to goalState.position, assign 0.0 to goalState.velocity, assign ControlState.HOLD to controlState
    }

    @Override
    public void update() {
        super.update();
        // TODO: call wristState.withPosition passing in position.mut_setMagnitude(primaryMotor.getEncoder().getPosition()
        // TODO: call wristState.withVelocity passing in velocity.mut_setMagnitude(primaryMotor.getEncoder().getVelocity()
        // TODO: call wristState.withTimeStamp passing in timestamp.mut_setMagnitude(Timer.getFPGATimestamp())
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
        // TODO: call primaryMotor.getEncoder()'s setPosition method passing in 0.0
        // TODO: do the same for followerMotor
    }

    private void applyVelocity() {
        // TODO: assign velocityProfile.calculate(goalState.velocity) to a variable called nextVelocitySetpoint
        // TODO: assign lastState.velocity to a variable called lastVelocitySetpoint
        // TODO: call feedforward's calculateWithVelocities method passing in lastVelocitySetpoint and nextVelocitySetpoint and assign to arbFeedforward
        // TODO: call primaryMotor.getClosedLoopController's setReference method passing in nextVelocitySetpoint, SparkBase.ControlType.kVelocity, velocityClosedLoopSlot, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        // TODO: call primaryMotor.getEncoder()'s getPosition() method and assign to lastState.position
        // TODO: assign nextVelocitySetpoint to lastState.velocity
    }

    private void applyPosition() {
        // TODO: assign lastState.velocity to a variable called lastVelocitySetpoint
        // TODO: call positionProfile's calculate method and passin profilePeriod.baseUnitMagnitude(), lastState, goalState) and assign to lastState
        // TODO: assign lastState.velocity to a variable called nextVelocitySetpoint
        // TODO: assign lastState.position to a variable called nextPositionSetpoint
        // TODO: call feedforward's calculateWithVelocities method passing in lastVelocitySetpoint and nextVelocitySetpoint and assign to arbFeedforward
        // TODO: call primaryMotor.getClosedLoopController's setReference method passing in nextPositionSetpoint, SparkBase.ControlType.kPosition, positionClosedLoopSlot, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        // TODO: call velocityProfile's reset method passing in nextVelocitySetpoint
    }
}
