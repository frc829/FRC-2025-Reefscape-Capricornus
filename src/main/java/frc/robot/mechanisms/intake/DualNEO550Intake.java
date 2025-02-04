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
    public void setVelocities(LinearVelocity... velocity) {

    }

    @Override
    public void updateSimState(double dtSeconds, double supplyVoltage) {

    }

    @Override
    public void setControl(IntakeMotorRequest... request) {

    }

    @Override
    public List<IntakeMotorState> getStates() {
        return intakeMotorStates;
    }

    @Override
    public List<IntakeMotorState> getStatesCopy() {
        return intakeMotorStates
                .stream()
                .map(state -> state.clone())
                .toList();
    }

    @Override
    public List<IntakeMotorState> getLastIntakeStates() {
        return lastIntakeMotorStates;
    }

    @Override
    public boolean hasElement() {
        return false;
    }

    @Override
    public void update() {
        lastIntakeMotor0State.withIntakeState(intakeMotor0State);
        lastIntakeMotor1State.withIntakeState(intakeMotor1State);
        updateState();
        updateTelemetry();
    }

    private void updateState() {
        intakeMotor0State.withVelocity(velocity0.mut_setMagnitude(motor0.getEncoder().getVelocity()))
                .withTimestamp(timestamp0.mut_setMagnitude(Timer.getFPGATimestamp()));
        intakeMotor1State.withVelocity(velocity1.mut_setMagnitude(motor1.getEncoder().getVelocity()))
                .withTimestamp(timestamp1.mut_setMagnitude(Timer.getFPGATimestamp()));
    }


    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }
}
