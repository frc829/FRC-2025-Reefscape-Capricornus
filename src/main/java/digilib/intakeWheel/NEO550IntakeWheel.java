package digilib.intakeWheel;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import digilib.elevator.ElevatorTelemetry;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class NEO550IntakeWheel implements IntakeWheel {
    private final IntakeWheelState lastState = new IntakeWheelState();
    private final IntakeWheelState state = new IntakeWheelState();
    private final LinearVelocity maxVelocity;
    private final IntakeWheelTelemetry telemetry;
    private IntakeWheelRequest intakeWheelRequest;
    private final SparkMax motor;
    private final SparkBaseConfig config;
    private final SlewRateLimiter profile;
    private final SimpleMotorFeedforward feedforward;
    private final Time profilePeriod;
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final MutLinearVelocity goalVelocity = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity lastVelocity = MetersPerSecond.mutable(0.0);
    private FlywheelSim flywheelSim = null;
    private SparkMaxSim sparkMaxSim = null;

    public NEO550IntakeWheel(IntakeWheelConstants constants, SparkMax motor, SparkBaseConfig config) {
        this.motor = motor;
        this.config = config;
        this.maxVelocity = constants.getMaxVelocity();
        this.feedforward = new SimpleMotorFeedforward(constants.getKs().baseUnitMagnitude(), constants.getKv().baseUnitMagnitude(), constants.getKa().baseUnitMagnitude(), constants.getUpdatePeriod().baseUnitMagnitude());
        this.profile = new SlewRateLimiter(constants.getMaxAcceleration().baseUnitMagnitude());
        this.profilePeriod = constants.getUpdatePeriod();
        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeo550(1);
            sparkMaxSim = new SparkMaxSim(motor, dcMotor);
            LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(
                    constants.getKv().baseUnitMagnitude(),
                    constants.getKa().baseUnitMagnitude());
            flywheelSim = new FlywheelSim(
                    plant,
                    dcMotor);
        }
        this.telemetry = new IntakeWheelTelemetry(
                constants.getName(),
                constants.getWheelRadius(),
                constants.getMaxVelocity(),
                constants.getMaxAcceleration());
    }

    @Override
    public boolean setNeutralModeToBrake() {
        config.idleMode(kBrake);
        REVLibError status = motor.configureAsync(config, kNoResetSafeParameters, kPersistParameters);
        return status == REVLibError.kOk;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        config.idleMode(kCoast);
        REVLibError status = motor.configureAsync(config, kNoResetSafeParameters, kPersistParameters);
        return status == REVLibError.kOk;
    }

    @Override
    public void setControl(IntakeWheelRequest request) {
        if (intakeWheelRequest != request) {
            intakeWheelRequest = request;
        }
        request.apply(this);
    }

    @Override
    public IntakeWheelState getState() {
        return state;
    }

    @Override
    public IntakeWheelState getStateCopy() {
        return state.clone();
    }

    @Override
    public IntakeWheelState getLastIntakeState() {
        return lastState;
    }

    @Override
    public void setVelocity(LinearVelocity velocity) {
        goalVelocity.mut_replace(velocity);
        double nextVelocitySetpoint = profile.calculate(goalVelocity.baseUnitMagnitude());
        double lastVelocitySetpoint = lastVelocity.baseUnitMagnitude();
        double arbFeedforward = feedforward.calculateWithVelocities(lastVelocitySetpoint, nextVelocitySetpoint);
        motor.getClosedLoopController().setReference(nextVelocitySetpoint, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot1, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
        lastVelocity.mut_setMagnitude(nextVelocitySetpoint);
    }

    @Override
    public void update() {
        lastState.withIntakeState(state);
        updateState();
        updateTelemetry();
    }

    private void updateState() {
        state.withVelocity(velocity.mut_setMagnitude(motor.getEncoder().getVelocity()))
                .withTimestamp(timestamp.mut_setMagnitude(Timer.getFPGATimestamp()));
    }


    @Override
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        double inputVoltage = motor.getAppliedOutput() * 12.0;
        flywheelSim.setInputVoltage(inputVoltage);
        flywheelSim.update(dtSeconds);
        sparkMaxSim.iterate(flywheelSim.getOutput(0), 12.0, dtSeconds);
    }


    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public LinearVelocity getMaxVelocity() {
        return maxVelocity;
    }
}