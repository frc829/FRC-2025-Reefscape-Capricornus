package frc.robot.mechanisms.intake;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.mechanisms.arm.ArmState;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class DualNEO550Intake implements Intake {

    private final IntakeMotorState lastIntakeMotor0State = new IntakeMotorState();
    private final IntakeMotorState lastIntakeMotor1State = new IntakeMotorState();
    private final IntakeMotorState intakeMotor0State = new IntakeMotorState();
    private final IntakeMotorState intakeMotor1State = new IntakeMotorState();
    private final List<IntakeMotorState> lastIntakeMotorStates = List.of(lastIntakeMotor0State, lastIntakeMotor1State);
    private final List<IntakeMotorState> intakeMotorStates = List.of(intakeMotor0State, intakeMotor1State);
    private IntakeMotorRequest intakeMotor0Request;
    private IntakeMotorRequest intakeMotor1Request;
    private final IntakeMotorConstants intakeMotor0Constants;
    private final IntakeMotorConstants intakeMotor1Constants;
    private final SparkMax motor0;
    private final SparkMax motor1;
    private final SparkBaseConfig motor0Config;
    private final SparkBaseConfig motor1Config;
    private final SlewRateLimiter motor0Profile;
    private final SlewRateLimiter motor1Profile;
    private final SimpleMotorFeedforward feedforward0;
    private final SimpleMotorFeedforward feedforward1;
    private final Time profilePeriod;
    private final MutLinearVelocity velocity0 = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity velocity1 = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp0 = Seconds.mutable(0.0);
    private final MutTime timestamp1 = Seconds.mutable(0.0);
    private final MutLinearVelocity goalVelocity0 = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity goalVelocity1 = MetersPerSecond.mutable(0.0);
    private DCMotorSim simIntakeMotor0 = null;
    private DCMotorSim simIntakeMotor1 = null;
    private SparkMaxSim sparkMax0Sim = null;
    private SparkMaxSim sparkMax1Sim = null;

    public DualNEO550Intake(
            IntakeMotorConstants intakeMotor0Constants,
            IntakeMotorConstants intakeMotor1Constants,
            SparkMax motor0,
            SparkMax motor1,
            SparkBaseConfig motor0Config,
            SparkBaseConfig motor1Config,
            Time updatePeriod) {
        this.intakeMotor0Constants = intakeMotor0Constants;
        this.intakeMotor1Constants = intakeMotor1Constants;
        this.motor0 = motor0;
        this.motor1 = motor1;
        this.motor0Config = motor0Config;
        this.motor1Config = motor1Config;
        this.feedforward0 = new SimpleMotorFeedforward(
                intakeMotor0Constants.getKs().baseUnitMagnitude(),
                intakeMotor0Constants.getKv().baseUnitMagnitude(),
                intakeMotor0Constants.getKa().baseUnitMagnitude(),
                updatePeriod.baseUnitMagnitude());
        this.feedforward1 = new SimpleMotorFeedforward(
                intakeMotor1Constants.getKs().baseUnitMagnitude(),
                intakeMotor1Constants.getKv().baseUnitMagnitude(),
                intakeMotor1Constants.getKa().baseUnitMagnitude(),
                updatePeriod.baseUnitMagnitude());
        this.motor0Profile = new SlewRateLimiter(intakeMotor0Constants.getMaxAngularAcceleration().baseUnitMagnitude());
        this.motor1Profile = new SlewRateLimiter(intakeMotor1Constants.getMaxAngularAcceleration().baseUnitMagnitude());
        this.profilePeriod = updatePeriod;


    }

    @Override
    public boolean setNeutralModeToBrake() {
        // TODO: call motor0Configs's idleMode method and pass in kBrake
        // TODO: call motor1Config's idleMode method and pass in kBrake
        // TODO: create a REVLibError variable called motor0ConfigStatus assign motor0.configureAsync passing in motor0Config, kNoResetSafeParameters, and  kPersistParameters
        // TODO: create a REVLibError variable called motor1ConfigStatus assign motor1.configureAsync passing in motor1ConfigStatus, kNoResetSafeParameters, and  kPersistParameters
        // TODO: return motor0ConfigStatus == REVLibError.kOk && motor1ConfigStatus == REVLibError.kOk;
        return false;  // TODO: remove this when done.  Since you've returned in the previous line.
    }

    @Override
    public boolean setNeutralModeToCoast() {
        // TODO: identical to setNeutralModeToBrake but with kCoast instead of kBrake
        return false;  // TODO: remove this when done.  Since you've returned in the previous line.
    }

    @Override
    public List<IntakeMotorState> getStates() {
        return intakeMotorStates;
    }

    @Override
    public List<IntakeMotorState> getStatesCopy() {

    }

    @Override
    public List<IntakeMotorState> getLastIntakeStates() {
        return List.of();
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
        // TODO: assign motor0Profile.calculate(goalVelocity0.baseUnitMagnitude()) to a variable called nextVelocity0Setpoint
        // TODO: assign motor1Profile.calculate(goalVelocity1.baseUnitMagnitude()) to a variable called nextVelocity1Setpoint
        // TODO: assign lastVelocity0.baseUnitMagnitude() to a variable called lastVelocity0Setpoint;
        // TODO: assign lastVelocity1.baseUnitMagnitude() to a variable called lastVelocity1Setpoint;
        // TODO: call motor0Feedforward's calculate method passing in lastVelocity0Setpoint and nextVelocity0Setpoint and assign to arbFeedforward0
        // TODO: call motor1Feedforward's calculate method passing in lastVelocity1Setpoint and nextVelocity1Setpoint and assign to arbFeedforward1
        // TODO: call motor0.getClosedLoopController's setReference method passing in nextVelocity0Setpoint, SparkBase.ControlType.kVelocity, motor0ClosedLoopSlot, arbFeedforward0, SparkClosedLoopController.ArbFFUnits.kVoltage;
        // TODO: call motor1.getClosedLoopController's setReference method passing in nextVelocity1Setpoint, SparkBase.ControlType.kVelocity, motor1ClosedLoopSlot, arbFeedforward1, SparkClosedLoopController.ArbFFUnits.kVoltage;
        // TODO: call lastVelocity0.mut_setMagnitude and pass in nextVelocity0Setpoint
        // TODO: call lastVelocity1.mut_setMagnitude and pass in nextVelocity1Setpoint
    }
}
