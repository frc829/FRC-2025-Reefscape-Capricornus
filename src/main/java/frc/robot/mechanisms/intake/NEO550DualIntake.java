package frc.robot.mechanisms.intake;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class NEO550DualIntake extends DualIntake {

    private final SparkMax motor0;
    private final SparkMax motor1;
    private final SparkBaseConfig motor0Config;
    private final SparkBaseConfig motor1Config;
    private final SlewRateLimiter motor0Profile;
    private final SlewRateLimiter motor1Profile;
    private final ClosedLoopSlot motor0ClosedLoopSlot;
    private final ClosedLoopSlot motor1ClosedLoopSlot;
    private final SimpleMotorFeedforward motor0Feedforward;
    private final SimpleMotorFeedforward motor1Feedforward;
    private final Time profilePeriod;
    private final MutLinearVelocity intakeVelocity0 = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity intakeVelocity1 = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final MutLinearVelocity goalVelocity0 = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity goalVelocity1 = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity lastVelocity0 = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity lastVelocity1 = MetersPerSecond.mutable(0.0);
    private ControlState controlState = ControlState.IDLE;


    public NEO550DualIntake(
            DualIntakeControlParameters dualIntakeControlParameters,
            SparkMax motor0,
            SparkMax motor1,
            SparkBaseConfig motor0Config,
            SparkBaseConfig motor1Config,
            ClosedLoopSlot motor0ClosedLoopSlot,
            ClosedLoopSlot motor1ClosedLoopSlot) {
        super(dualIntakeControlParameters);
        this.motor0 = motor0;
        this.motor1 = motor1;
        this.motor0Config = motor0Config;
        this.motor1Config = motor1Config;
        this.motor0ClosedLoopSlot = motor0ClosedLoopSlot;
        this.motor1ClosedLoopSlot = motor1ClosedLoopSlot;
        this.motor0Feedforward = new SimpleMotorFeedforward(
                dualIntakeControlParameters.getIntake0ks().baseUnitMagnitude(),
                dualIntakeControlParameters.getIntake0kv().baseUnitMagnitude(),
                dualIntakeControlParameters.getIntake0ka().baseUnitMagnitude(),
                dualIntakeControlParameters.getUpdatePeriod().baseUnitMagnitude());
        this.motor1Feedforward = new SimpleMotorFeedforward(
                dualIntakeControlParameters.getIntake1ks().baseUnitMagnitude(),
                dualIntakeControlParameters.getIntake1kv().baseUnitMagnitude(),
                dualIntakeControlParameters.getIntake1ka().baseUnitMagnitude(),
                dualIntakeControlParameters.getUpdatePeriod().baseUnitMagnitude());
        double motor0MaxAcceleration = motor0Feedforward.maxAchievableAcceleration(12.0, 0.0);
        double motor1MaxAcceleration = motor1Feedforward.maxAchievableAcceleration(12.0, 0.0);
        this.motor0Profile = new SlewRateLimiter(motor0MaxAcceleration);
        this.motor1Profile = new SlewRateLimiter(motor1MaxAcceleration);
        this.profilePeriod = dualIntakeControlParameters.getUpdatePeriod();


    }

    @Override
    public boolean setNeutralModeToBrake() {
        motor0Config.idleMode(kBrake);
        motor1Config.idleMode(kBrake);
        REVLibError motor0ConfigStatus = motor0.configureAsync(motor0Config, kNoResetSafeParameters, kPersistParameters);
        REVLibError motor1ConfigStatus = motor1.configureAsync(motor1Config, kNoResetSafeParameters, kPersistParameters);
        return motor0ConfigStatus == REVLibError.kOk && motor1ConfigStatus == REVLibError.kOk;
    }

    private void motor0Config() {
    }

    @Override
    public boolean setNeutralModeToCoast() {
        motor0Config.idleMode(kCoast);
        motor1Config.idleMode(kCoast);
        REVLibError motor0ConfigStatus = motor0.configureAsync(motor0Config, kNoResetSafeParameters, kPersistParameters);
        REVLibError motor1ConfigStatus = motor1.configureAsync(motor1Config, kNoResetSafeParameters, kPersistParameters);
        return motor0ConfigStatus == REVLibError.kOk && motor1ConfigStatus == REVLibError.kOk;
    }

    @Override
    public void setVelocity(LinearVelocity velocity0, LinearVelocity velocity1) {
        goalVelocity0.mut_replace(velocity0);
        goalVelocity1.mut_replace(velocity1);
        controlState = ControlState.VELOCITY;
    }

    @Override
    public void setIdle() {
        if (controlState != ControlState.IDLE) {
            controlState = ControlState.IDLE;
            goalVelocity0.mut_replace(MetersPerSecond.of(0.0));
            goalVelocity1.mut_replace(MetersPerSecond.of(0.0));
        }
    }

    @Override
    public void update() {
        super.update();
        dualIntakeState.withVelocity(
                intakeVelocity0.mut_setMagnitude(motor0.getEncoder().getVelocity()),
                intakeVelocity1.mut_setMagnitude(motor1.getEncoder().getVelocity()));
        dualIntakeState.withTimestamp(timestamp.mut_setMagnitude(Timer.getFPGATimestamp()));
        applyVelocity();
    }


    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }

    private void applyVelocity() {
        var nextVelocity0Setpoint = motor0Profile.calculate((goalVelocity0.baseUnitMagnitude()));
        var nextVelocity1Setpoint = motor1Profile.calculate((goalVelocity1.baseUnitMagnitude()));
        var lastVelocity0Setpoint = lastVelocity0.baseUnitMagnitude();
        var lastVelocity1Setpoint = lastVelocity1.baseUnitMagnitude();
        double arbFeedforward0 = motor0Feedforward.calculateWithVelocities(lastVelocity0Setpoint, nextVelocity0Setpoint);
        double arbFeedforward1 = motor1Feedforward.calculateWithVelocities(lastVelocity1Setpoint, nextVelocity1Setpoint);
        motor0.getClosedLoopController().setReference(nextVelocity0Setpoint, SparkBase.ControlType.kVelocity, motor0ClosedLoopSlot, arbFeedforward0, SparkClosedLoopController.ArbFFUnits.kVoltage);
        motor1.getClosedLoopController().setReference(nextVelocity1Setpoint, SparkBase.ControlType.kVelocity, motor1ClosedLoopSlot, arbFeedforward1, SparkClosedLoopController.ArbFFUnits.kVoltage);
        lastVelocity0.mut_setMagnitude(nextVelocity0Setpoint);
        lastVelocity1.mut_setMagnitude(nextVelocity1Setpoint);
    }
}