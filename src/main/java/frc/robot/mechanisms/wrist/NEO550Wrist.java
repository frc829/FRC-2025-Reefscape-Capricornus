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

import javax.naming.ldap.Control;

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
    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
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
        primaryMotorConfig.idleMode(kBrake);
        REVLibError primaryMotorConfigStatus = primaryMotor.configureAsync(primaryMotorConfig, kNoResetSafeParameters, kPersistParameters);
        return primaryMotorConfigStatus == REVLibError.kOk;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        primaryMotorConfig.idleMode(kCoast);
        REVLibError primaryMotorConfigStatus = primaryMotor.configureAsync(primaryMotorConfig, kNoResetSafeParameters, kPersistParameters);
        return primaryMotorConfigStatus == REVLibError.kOk;
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        goalState.velocity = velocity.baseUnitMagnitude();
        controlState = ControlState.VELOCITY;
    }

    @Override
    public void setPosition(Angle position) {
    goalState.position = position.baseUnitMagnitude();
    goalState.velocity = 0.0;
    controlState = ControlState.POSITION;
    }

    @Override
    public void setHold() {
        if(controlState != ControlState.HOLD) {
            goalState.position = primaryMotor.getEncoder().getPosition();
            controlState = ControlState.HOLD;
        }
    }

    @Override
    public void update() {
        super.update();
        wristState.withPosition(position.mut_setBaseUnitMagnitude(primaryMotor.getEncoder().getPosition()));
        wristState.withVelocity(velocity.mut_setBaseUnitMagnitude(primaryMotor.getEncoder().getVelocity()));
        wristState.withTimestamp(timestamp.mut_setBaseUnitMagnitude(Timer.getFPGATimestamp()));
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
    }

    private void applyVelocity() {
        var nextVelocitySetpoint = velocityProfile.calculate(goalState.velocity);
        var lastVelocitySetpoint = lastState.velocity;
        var arbFeedforward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetpoint);
        primaryMotor.getClosedLoopController().setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, velocityClosedLoopSlot, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        primaryMotor.getEncoder().setPosition(lastState.position);
        lastState.position = nextVelocitySetpoint;
    }

    private void applyPosition() {
        var lastVelocitySetpoint = lastState.velocity;
        lastState = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), lastState, goalState);
        var nextVelocitySetPoint = lastState.velocity;
        var nextPositionsetPoint = lastState.position;
        var arbFeedForward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetPoint);
        primaryMotor.getClosedLoopController().setReference(nextPositionsetPoint, SparkBase.ControlType.kPosition, positionClosedLoopSlot, arbFeedForward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        velocityProfile.reset(nextVelocitySetPoint);
    }
}
