package frc.robot.mechanisms.arm;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.ClosedLoopSlot.kSlot1;
import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
import static edu.wpi.first.units.Units.*;

public class NEO550Arm implements Arm {

    private final ArmState lastArmState = new ArmState();
    private final ArmState armState = new ArmState();
    private final Angle minAngle;
    private final Angle maxAngle;
    private final ArmTelemetry armTelemetry;
    private ArmRequest armRequest;
    private final ArmConstants armConstants;
    private final SparkMax motor;
    private final SparkBaseConfig motorConfig;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ExponentialProfile.State goalState = new ExponentialProfile.State();
    private final ArmFeedforward feedforward;
    private final Time profilePeriod;
    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private ExponentialProfile.State lastState = new ExponentialProfile.State();
    private SingleJointedArmSim simArm = null;
    private SparkMaxSim sparkMaxSim = null;
    private boolean hold = false;

    public NEO550Arm(
            ArmConstants armConstants,
            SparkMax motor,
            SparkBaseConfig motorConfig,
            Time updatePeriod) {
        this.minAngle = armConstants.getMinAngle();
        this.maxAngle = armConstants.getMaxAngle();
        this.armConstants = armConstants;
        this.motor = motor;
        this.motorConfig = motorConfig;
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
        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNeo550(1);
            LinearSystem<N2, N1, N2> plant = LinearSystemId
                    .identifyPositionSystem(armConstants.getKv().baseUnitMagnitude(),
                            armConstants.getKa().baseUnitMagnitude());
            sparkMaxSim = new SparkMaxSim(motor, gearbox);
            this.simArm = new SingleJointedArmSim(
                    plant,
                    gearbox,
                    armConstants.getReduction(),
                    armConstants.getArmLength().baseUnitMagnitude(),
                    minAngle.baseUnitMagnitude(),
                    maxAngle.baseUnitMagnitude(),
                    true,
                    armConstants.getStartingAngle().baseUnitMagnitude());
        }
        this.armTelemetry = new ArmTelemetry(
                "Hook",
                armConstants.getMinAngle(),
                armConstants.getMaxAngle(),
                armConstants.getMaxAngularVelocity(),
                armConstants.getMaxAngularAcceleration());

    }

    @Override
    public boolean setNeutralModeToBrake() {
        motorConfig.idleMode(kBrake);
        REVLibError motorConfigStatus = motor.configureAsync(motorConfig, kNoResetSafeParameters, kPersistParameters);
        return motorConfigStatus == REVLibError.kOk;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        motorConfig.idleMode(kCoast);
        REVLibError motorConfigStatus = motor.configureAsync(motorConfig, kNoResetSafeParameters, kPersistParameters);
        return motorConfigStatus == REVLibError.kOk;
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
    public void setControl(ArmRequest request) {
        if (armRequest != request) {
            armRequest = request;
        }
        request.apply(this);
    }


    @Override
    public void setPosition(Angle position) {
        goalState.position = position.baseUnitMagnitude();
        goalState.velocity = 0.0;
        double lastVelocitySetpoint = lastState.velocity;
        lastState = positionProfile.calculate(profilePeriod.baseUnitMagnitude(), lastState, goalState);
        double nextVelocitySetpoint = lastState.velocity;
        double nextPositionSetpoint = lastState.position;
        double currentPosition = motor.getEncoder().getPosition();
        double arbFeedfoward = feedforward.calculateWithVelocities(currentPosition, lastVelocitySetpoint, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextPositionSetpoint, SparkBase.ControlType.kPosition, kSlot0, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        velocityProfile.reset(nextVelocitySetpoint);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        goalState.velocity = velocity.baseUnitMagnitude();
        double nextVelocitySetpoint = velocityProfile.calculate(goalState.velocity);
        double lastVelocitySetPoint = lastState.velocity;
        double currentPosition = motor.getEncoder().getPosition();
        double arbFeedfoward = feedforward.calculateWithVelocities(currentPosition, lastVelocitySetPoint, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, kSlot1, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        lastState.position = motor.getEncoder().getPosition();
        lastState.velocity = nextVelocitySetpoint;
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(0.0);
    }


    @Override
    public void update() {
        lastArmState.withArmState(armState);
        updateState();
        updateTelemetry();
    }

    private void updateState() {
        armState.withPosition(position.mut_setMagnitude(motor.getEncoder().getPosition()))
                .withVelocity(velocity.mut_setMagnitude(motor.getEncoder().getVelocity()))
                .withTimestamp(timestamp.mut_setMagnitude(Timer.getFPGATimestamp()));
    }

    @Override
    public void updateTelemetry() {
        armTelemetry.telemeterize(armState);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getAppliedOutput() * 12.0;
        simArm.setInputVoltage(inputVoltage);
        simArm.update(dt);
        sparkMaxSim.iterate(simArm.getVelocityRadPerSec(), 12.0, dt);
    }

    @Override
    public Angle getMaxAngle() {
        return maxAngle;
    }

    @Override
    public Angle getMinAngle() {
        return minAngle;
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
