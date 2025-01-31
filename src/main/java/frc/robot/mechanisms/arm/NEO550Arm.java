package frc.robot.mechanisms.arm;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class NEO550Arm implements Arm {

    private final ArmState lastArmState = new ArmState();
    private final ArmState armState = new ArmState();
    private final Angle minAngle;
    private final Angle maxAngle;
    private ArmRequest armRequest;
    private final SparkMax motor;
    private final SparkBaseConfig motorConfig;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ClosedLoopSlot positionClosedLoopSlot;
    private final ClosedLoopSlot velocityClosedLoopSlot;
    private final ExponentialProfile.State goalState = new ExponentialProfile.State();
    private final ArmFeedforward feedforward;
    private final Time profilePeriod;
    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private ExponentialProfile.State lastState = new ExponentialProfile.State();
    private ControlState controlState = ControlState.VELOCITY;

    public NEO550Arm(
            ArmConstants armConstants,
            SparkMax motor,
            SparkBaseConfig motorConfig,
            ClosedLoopSlot positionClosedLoopSlot,
            ClosedLoopSlot velocityClosedLoopSlot,
            Time updatePeriod) {
        this.minAngle = armConstants.getMinAngle();
        this.maxAngle = armConstants.getMaxAngle();
        this.motor = motor;
        this.motorConfig = motorConfig;
        this.positionClosedLoopSlot = positionClosedLoopSlot;
        this.velocityClosedLoopSlot = velocityClosedLoopSlot;
        this.feedforward = new ArmFeedforward(
                armConstants.getKs().baseUnitMagnitude(),
                armConstants.getKg().baseUnitMagnitude(),
                armConstants.getKv().baseUnitMagnitude(),
                armConstants.getKa().baseUnitMagnitude(),
                updatePeriod.baseUnitMagnitude());
        this.positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        12.0,
                        armConstants.getKv().baseUnitMagnitude(),
                        armConstants.getKa().baseUnitMagnitude()));
        double maxAcceleration = feedforward.maxAchievableAcceleration(12.0, Math.PI / 2, 0.0);
        this.velocityProfile = new SlewRateLimiter(maxAcceleration);
        this.profilePeriod = updatePeriod;

    }

    @Override
    public boolean setNeutralModeToBrake() {
        // TODO: call motorConfig's idleMode method and pass in kBrake
        // TODO: create a REVLibError variable called motorConfigStatus assign primaryMotor.configureAsync passing in motorConfig, kNoResetSafeParameters, and  kPersistParameters
        // TODO: return motorConfig == REVLibError.kOk ;
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
        lastArmState.withArmState(armState);
        updateState();
        switch (controlState) {
            case VELOCITY -> applyVelocity();
            case POSITION, HOLD -> applyPosition();
        }
    }

    @Override
    public ArmRequest createHoldRequest() {
        return new ArmRequest.Hold();
    }

    @Override
    public ArmRequest createPositionRequest() {
        return new ArmRequest.Position(minAngle, maxAngle);
    }

    @Override
    public ArmRequest createVelocityRequest() {
        return new ArmRequest.Velocity(minAngle, maxAngle);
    }

    private void updateState() {
        // TODO: call armState.withPosition passing in position.mut_setMagnitude(primaryMotor.getEncoder().getPosition()
        // TODO: call armState.withVelocity passing in velocity.mut_setMagnitude(primaryMotor.getEncoder().getVelocity()
        // TODO: call armState.withTimeStamp passing in timestamp.mut_setMagnitude(Timer.getFPGATimestamp())
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }

    @Override
    public void updateSimState(Time dt, Voltage supplyVoltage) {
        // TODO: will do later
    }

    @Override
    public void setControl(ArmRequest request) {
        if(armRequest != request){
            armRequest = request;
        }
        request.apply(this);
    }

    @Override
    public ArmState getState() {
        return armState;
    }

    @Override
    public ArmState getStateCopy() {
        return armState.clone();
    }

    @Override
    public ArmState getLastArmState() {
        return lastArmState;
    }

    @Override
    public void resetPosition() {
        // TODO: call motor.getEncoder()'s setPosition method passing in 0.0
        // TODO: do the same for followerMotor
    }

    private void applyVelocity() {
        // TODO: assign velocityProfile.calculate(goalState.velocity) to a variable called nextVelocitySetpoint
        // TODO: assign lastState.velocity to a variable called lastVelocitySetpoint
        // TODO: call feedforward's calculateWithVelocities method passing in lastVelocitySetpoint and nextVelocitySetpoint and assign to arbFeedforward
        // TODO: call motor.getClosedLoopController's setReference method passing in nextVelocitySetpoint, SparkBase.ControlType.kVelocity, velocityClosedLoopSlot, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        // TODO: call motor.getEncoder()'s getPosition() method and assign to lastState.position
        // TODO: assign nextVelocitySetpoint to lastState.velocity
    }

    private void applyPosition() {
        // TODO: assign lastState.velocity to a variable called lastVelocitySetpoint
        // TODO: call positionProfile's calculate method and passing profilePeriod.baseUnitMagnitude(), lastState, goalState) and assign to lastState
        // TODO: assign lastState.velocity to a variable called nextVelocitySetpoint
        // TODO: assign lastState.position to a variable called nextPositionSetpoint
        // TODO: call feedforward's calculateWithVelocities method passing in lastVelocitySetpoint and nextVelocitySetpoint and assign to arbFeedforward
        // TODO: call motor.getClosedLoopController's setReference method passing in nextPositionSetpoint, SparkBase.ControlType.kPosition, positionClosedLoopSlot, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        // TODO: call velocityProfile's reset method passing in nextVelocitySetpoint
    }
}
