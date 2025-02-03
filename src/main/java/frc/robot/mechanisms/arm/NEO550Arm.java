package frc.robot.mechanisms.arm;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.*;

import static com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters;
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
    private boolean hold = false;

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
        motorConfig.idleMode(kBrake);
        REVLibError motorConfigStatus = motor.configureAsync(motorConfig, kNoResetSafeParameters, kPersistParameters);
        return motorConfig == REVLibError.kOk;
    }

    @Override
    public boolean setNeutralModeToCoast() {
            motorConfig.idleMode(kCoast);
            REVLibError motorConfigStatus = primaryMotor.configureAsync(motorConfig, kNoResetSafeParameters, kPersistParameters);
            return motorConfig == REVLibError.kOk;
    }

    @Override
    public void setVelocity(AngularVelocity velocity){
        goalState.velocity = velocity.baseUnitMagnitude();
        controlState = ControlState.VELOCITY;

    }

    @Override
    public void setPosition(Angle position) {
        goalState.position = position.baseUnitMagnitude();
        goalState.velocity = 0.0;
        controlState = .POSITION;
    }

    @Override
    public void update() {
        lastArmState.withArmState(armState);
        updateState();
    }

    private void updateState() {
        armState.withPosition(position.mut_setMagnitude(motor.getEncoder().getPosition()));
        armState.withVelocity(velocity.mut_setMagnitude(primaryMotor.getEncoder().getVelocity()));
        armState.withTimeStamp(timestamp.mut_setMagnitude(Timer.getFPGATimestamp()));
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
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
        motor.getEncoder().setPosition(0.0);
    }

    private void applyVelocity() {
        nextVelocitySetpoint = velocityProfile.calculate(goalState.velocity);
        lastVelocitySetpoint = lastState.velocity;
        arbFeedforward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetpoint);
        motor.getClosedLoopController.setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, velocityClosedLoopSlot, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        lastState.position = motor.getEncoder().getPosition();
        lastState.velocity = nextVelocitySetpoint;
    }

    private void applyPosition() {
        lastVelocitySetpoint = lastState.velocity;
        lastState = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), lastState, goalState);
        nextVelocitySetpoint = lastState.velocity;
        nextPositionSetpoint = lastState.position;
        arbFeedforward = feedforward.calculateWithVelocities(lastVelocitySetpoint , nextVelocitySetpoint);
        motor.getClosedLoopController.setReference(nextPositionSetpoint, SparkBase.ControlType.kPosition, positionClosedLoopSlot, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        velocityProfile.reset(nextVelocitySetpoint);
    }

    @Override
    public void enableHold() {
        hold = true;
    }

    @Override
    public void disableHold() {
        hold = false;
    }

    @Override
    public boolean isHoldEnabled() {
        return hold;
    }

    @Override
    public Angle getMaxAngle() {
        return maxAngle;
    }

    @Override
    public Angle getMinAngle() {
        return minAngle;
    }
}
