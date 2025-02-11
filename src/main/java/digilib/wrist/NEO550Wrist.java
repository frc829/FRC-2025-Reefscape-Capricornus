package digilib.wrist;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static com.revrobotics.spark.ClosedLoopSlot.kSlot0;
import static com.revrobotics.spark.ClosedLoopSlot.kSlot1;
import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;

public class NEO550Wrist implements Wrist {
    private final Angle minAngle;
    private final Angle maxAngle;
    private final WristState lastWristState = new WristState();
    private final WristState wristState = new WristState();
    private final WristTelemetry wristTelemetry;
    private DCMotorSim simWrist;
    private SparkMaxSim sparkMaxSim;
    private WristRequest wristRequest;
    private final SparkMax motor;
    private final SparkBaseConfig config;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ExponentialProfile.State goalState = new ExponentialProfile.State();
    private final SimpleMotorFeedforward feedforward;
    private final Time profilePeriod;
    private final MutAngle position = Radians.mutable(0.0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final CANcoder cancoder;
    private boolean hold = false;
    private AngularVelocity maxVelocity;

    private ExponentialProfile.State lastState = new ExponentialProfile.State();

    public NEO550Wrist(
            WristConstants wristConstants,
            SparkMax motor,
            SparkBaseConfig config,
            CANcoder cancoder,
            Time updatePeriod) {
        minAngle = wristConstants.getMinAngle();
        maxAngle = wristConstants.getMaxAngle();
        this.cancoder = cancoder;
        this.motor = motor;
        this.config = config;
        this.feedforward = new SimpleMotorFeedforward(
                wristConstants.getKs().baseUnitMagnitude(),
                wristConstants.getKv().baseUnitMagnitude(),
                wristConstants.getKa().baseUnitMagnitude(),
                updatePeriod.baseUnitMagnitude());
        this.positionProfile = new ExponentialProfile(
                ExponentialProfile.Constraints.fromCharacteristics(
                        12.0,
                        wristConstants.getKv().baseUnitMagnitude(),
                        wristConstants.getKa().baseUnitMagnitude()));
        double maxAcceleration = feedforward.maxAchievableAcceleration(12.0, 0.0);
        this.velocityProfile = new SlewRateLimiter(maxAcceleration);
        this.profilePeriod = updatePeriod;
        motor.getEncoder().setPosition(cancoder.getPosition().getValue().in(Radians));
        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeo550(1);
            LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(
                    wristConstants.getKv().baseUnitMagnitude(),
                    wristConstants.getKa().baseUnitMagnitude());
            this.sparkMaxSim = new SparkMaxSim(motor, dcMotor);
            this.simWrist = new DCMotorSim(
                    plant,
                    dcMotor,
                    Radians.of(0.0).baseUnitMagnitude(),
                    RadiansPerSecond.of(0.0).baseUnitMagnitude());
            sparkMaxSim.setPosition(wristConstants.getStartingAngle().baseUnitMagnitude());
        }
        this.maxVelocity = wristConstants.getMaxAngularVelocity();
        this.wristTelemetry = new WristTelemetry(
                "Wrist",
                wristConstants.getMinAngle(),
                wristConstants.getMaxAngle(),
                wristConstants.getMaxAngularVelocity(),
                wristConstants.getMaxAngularAcceleration());
    }

    @Override
    public boolean setNeutralModeToBrake() {
        config.idleMode(kBrake);
        REVLibError primaryMotorConfigStatus = motor.configureAsync(config, kNoResetSafeParameters, kPersistParameters);
        return primaryMotorConfigStatus == REVLibError.kOk;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        config.idleMode(kCoast);
        REVLibError primaryMotorConfigStatus = motor.configureAsync(config, kNoResetSafeParameters, kPersistParameters);
        return primaryMotorConfigStatus == REVLibError.kOk;
    }

    @Override
    public WristState getState() {
        return wristState;
    }

    @Override
    public WristState getStateCopy() {
        return wristState.clone();
    }

    @Override
    public WristState getLastArmState() {
        return lastWristState;
    }

    @Override
    public void setControl(WristRequest request) {
        if (wristRequest != request) {
            wristRequest = request;
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
        double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextPositionSetpoint, SparkBase.ControlType.kPosition, kSlot0, arbFeedfoward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        velocityProfile.reset(motor.getEncoder().getVelocity());
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        goalState.velocity = velocity.baseUnitMagnitude();
        double nextVelocitySetpoint = velocityProfile.calculate(goalState.velocity);
        double lastVelocitySetPoint = lastState.velocity;
        double arbFeedfoward = feedforward.calculateWithVelocities(lastVelocitySetPoint, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, kSlot1, arbFeedfoward);
        lastState.position = motor.getEncoder().getPosition();
        lastState.velocity = nextVelocitySetpoint;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.baseUnitMagnitude());
        velocityProfile.reset(motor.getEncoder().getVelocity());
        lastState.position = motor.getEncoder().getPosition();
        lastState.velocity = motor.getEncoder().getVelocity();
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(cancoder.getPosition().getValue().in(Radians));
        update();
    }

    @Override
    public void update() {
        lastWristState.withWristState(wristState);
        updateState();
        updateTelemetry();
    }

    private void updateState() {
        wristState.withPosition(position.mut_setBaseUnitMagnitude(motor.getEncoder().getPosition()));
        wristState.withVelocity(velocity.mut_setBaseUnitMagnitude(motor.getEncoder().getVelocity()));
        wristState.withTimestamp(timestamp.mut_setBaseUnitMagnitude(Timer.getFPGATimestamp()));
        wristState.withVoltage(motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @Override
    public void updateTelemetry() {
        wristTelemetry.telemeterize(wristState);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getAppliedOutput() * 12.0;
        simWrist.setInputVoltage(inputVoltage);
        simWrist.update(dt);
        sparkMaxSim.iterate(simWrist.getAngularVelocityRadPerSec(), 12.0, dt);
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
    public AngularVelocity getMaxVelocity() {
        return maxVelocity;
    }
}
