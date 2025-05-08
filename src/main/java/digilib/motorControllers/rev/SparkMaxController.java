package digilib.motorControllers.rev;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import digilib.motorControllers.MotorController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public abstract class SparkMaxController implements MotorController {

    protected final SparkMax sparkMax;
    protected final ExponentialProfile positionProfile;
    protected final TrapezoidProfile velocityProfile;
    private final ClosedLoopConfig.FeedbackSensor feedbackSensor;
    protected final DCMotor dcMotor;
    private ExponentialProfile.State lastPositionState = new ExponentialProfile.State();
    private final ExponentialProfile.State goalPositionState = new ExponentialProfile.State();
    private TrapezoidProfile.State lastVelocityState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State goalVelocityState = new TrapezoidProfile.State();


    protected SparkMaxController(
            SparkMax sparkMax,
            ExponentialProfile positionProfile,
            TrapezoidProfile velocityProfile,
            DCMotor dcMotor) {
        this.sparkMax = sparkMax;
        this.positionProfile = positionProfile;
        this.velocityProfile = velocityProfile;
        feedbackSensor = sparkMax.configAccessor.closedLoop.getFeedbackSensor();
        this.dcMotor = dcMotor;
    }

    @Override
    public final double getVoltage() {
        return sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    }

    @Override
    public final double getCurrentAmps() {
        return sparkMax.getOutputCurrent();
    }

    @Override
    public final double getPosition() throws IllegalCallerException {
        return switch (feedbackSensor) {
            case kPrimaryEncoder -> sparkMax.getEncoder().getPosition();
            case kAbsoluteEncoder -> sparkMax.getAbsoluteEncoder().getPosition();
            case kAlternateOrExternalEncoder -> sparkMax.getAlternateEncoder().getPosition();
            case kAnalogSensor -> sparkMax.getAnalog().getPosition();
            case kNoSensor -> throw new IllegalCallerException("No feedback sensor");
        };
    }

    @Override
    public final double getVelocity() {
        return switch (feedbackSensor) {
            case kPrimaryEncoder -> sparkMax.getEncoder().getVelocity();
            case kAbsoluteEncoder -> sparkMax.getAbsoluteEncoder().getVelocity();
            case kAlternateOrExternalEncoder -> sparkMax.getAlternateEncoder().getVelocity();
            case kAnalogSensor -> sparkMax.getAnalog().getVelocity();
            case kNoSensor -> throw new IllegalCallerException("No feedback sensor");
        };
    }

    @Override
    public final void applyVoltage(double voltage) {
        sparkMax.setVoltage(voltage);
        lastPositionState.position = getPosition();
        lastPositionState.velocity = getVelocity();
        lastVelocityState.position = getVelocity();
        lastVelocityState.velocity = getAcceleration();
    }

    @Override
    public final void applyCurrentAmps(double currentAmps) {
        sparkMax.getClosedLoopController().setReference(currentAmps, SparkBase.ControlType.kCurrent);
        lastPositionState.position = getPosition();
        lastPositionState.velocity = getVelocity();
        lastVelocityState.position = lastPositionState.velocity;
        lastVelocityState.velocity = getAcceleration();
    }

    @Override
    public final void applyPositionWithVoltage(double position) {
        goalPositionState.position = position;
        double lastVelocitySetpoint = lastPositionState.velocity;
        lastPositionState = positionProfile.calculate(0.020, lastPositionState, goalPositionState);
        double feedforwardVoltage = getFeedForward(lastVelocitySetpoint, lastPositionState.velocity);
        sparkMax.getClosedLoopController().setReference(
                lastPositionState.position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforwardVoltage,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
        lastVelocityState.position = lastPositionState.velocity;
        lastVelocityState.velocity = getAcceleration();
    }

    @Override
    public void applyPositionWithCurrent(double position) {

    }

    @Override
    public final void applyVelocityWithVoltage(double velocity) {
        goalVelocityState.position = velocity;
        double lastVelocitySetpoint = lastVelocityState.position;
        lastVelocityState = velocityProfile.calculate(0.020, lastVelocityState, goalVelocityState);
        double feedforwardVoltage = getFeedForward(lastVelocitySetpoint, lastPositionState.position);
        sparkMax.getClosedLoopController().setReference(
                lastVelocityState.position,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot1,
                feedforwardVoltage,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
        lastPositionState.position = getPosition();
        lastPositionState.velocity = lastVelocityState.position;
    }

    @Override
    public void applyVelocityWithCurrent(double velocity) {

    }

    protected abstract double getFeedForward(
            double currentVelocitySetpoint,
            double nextVelocitySetpoint);

    protected static final class SimThread {
        private static final double simLoopPeriodSeconds = 0.001;
        private final Notifier notifier;
        private double lastTimeSeconds = 0.0;

        public SimThread(
                SparkMaxSim sparkMaxSim,
                DCMotorSim dcMotorSim) {
            notifier = new Notifier(() -> {
                final double currentTimeSeconds = RobotController.getFPGATime() / 1_000_000.0;
                double deltaTime = currentTimeSeconds - lastTimeSeconds;
                lastTimeSeconds = currentTimeSeconds;
                double inputVoltage = sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
                dcMotorSim.setInputVoltage(inputVoltage);
                dcMotorSim.update(deltaTime);
                sparkMaxSim.iterate(dcMotorSim.getAngularVelocityRadPerSec(), RobotController.getBatteryVoltage(), deltaTime);
            });
            startPeriodic();
        }

        public SimThread(
                SparkMaxSim sparkMaxSim,
                ElevatorSim elevatorSim) {
            notifier = new Notifier(() -> {
                final double currentTimeSeconds = RobotController.getFPGATime() / 1_000_000.0;
                double deltaTime = currentTimeSeconds - lastTimeSeconds;
                lastTimeSeconds = currentTimeSeconds;
                double inputVoltage = sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
                elevatorSim.setInputVoltage(inputVoltage);
                elevatorSim.update(deltaTime);
                sparkMaxSim.iterate(elevatorSim.getVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), deltaTime);
            });
        }

        public SimThread(
                SparkMaxSim sparkMaxSim,
                SingleJointedArmSim armSim) {
            notifier = new Notifier(() -> {
                final double currentTimeSeconds = RobotController.getFPGATime() / 1_000_000.0;
                double deltaTime = currentTimeSeconds - lastTimeSeconds;
                lastTimeSeconds = currentTimeSeconds;
                double inputVoltage = sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
                armSim.setInputVoltage(inputVoltage);
                armSim.update(deltaTime);
                sparkMaxSim.iterate(armSim.getVelocityRadPerSec(), RobotController.getBatteryVoltage(), deltaTime);
            });
        }

        private void startPeriodic() {
            notifier.startPeriodic(simLoopPeriodSeconds);
        }
    }
}
