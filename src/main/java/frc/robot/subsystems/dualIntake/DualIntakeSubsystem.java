package frc.robot.subsystems.dualIntake;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import digilib.objectDetectors.ObjectDetector;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.intakeWheel.IntakeWheel;
import digilib.intakeWheel.IntakeWheelRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;

import static digilib.intakeWheel.IntakeWheelRequest.*;
import static edu.wpi.first.units.Units.*;

public class DualIntakeSubsystem implements Subsystem {
    private final IntakeWheel wheel0;
    private final IntakeWheel wheel1;
    private double wheel0LastSimTime;
    private double wheel1LastSimTime;
    private final ObjectDetector objectDetector;
    private final Time wheel0SimLoopPeriod;
    private final Time wheel1SimLoopPeriod;
    public final Trigger hasCoral;
    public final Trigger hasAlgae;

    public DualIntakeSubsystem(
            IntakeWheel wheel0,
            IntakeWheel wheel1,
            ObjectDetector objectDetector,
            Time wheel0SimLoopPeriod,
            Time wheel1SimLoopPeriod) {
        this.wheel0 = wheel0;
        this.wheel1 = wheel1;
        this.objectDetector = objectDetector;
        this.wheel0SimLoopPeriod = wheel0SimLoopPeriod;
        this.wheel1SimLoopPeriod = wheel1SimLoopPeriod;
        hasCoral = new Trigger(objectDetector.getState()::isInRange);
        hasAlgae = new Trigger(() -> wheel0.getState().getCurrent().gte(Amps.of(40.0)));

        SysIdRoutine.Config wheel0Config = new SysIdRoutine.Config(
                Volts.per(Second).of(1.0),
                Volts.of(7.0),
                Seconds.of(10.0));
        SysIdRoutine.Config wheel1Config = new SysIdRoutine.Config(
                Volts.per(Second).of(1.0),
                Volts.of(7.0),
                Seconds.of(10.0));
        VoltageRequest intakeWheel0VoltageRequest = new VoltageRequest();
        VoltageRequest intakeWheel1VoltageRequest = new VoltageRequest();
        SysIdRoutine.Mechanism wheel0Mechanism = new SysIdRoutine.Mechanism(
                volts -> wheel0.setControl(intakeWheel0VoltageRequest.withVoltage(volts)),
                log -> log
                        .motor("wheel0")
                        .angularVelocity(wheel0.getState().getAngularVelocity())
                        .voltage(wheel0.getState().getVoltage()),
                this,
                "wheel0-sysIdRoutine");
        SysIdRoutine.Mechanism wheel1Mechanism = new SysIdRoutine.Mechanism(
                volts -> wheel1.setControl(intakeWheel1VoltageRequest.withVoltage(volts)),
                log -> log
                        .motor("wheel1")
                        .angularVelocity(wheel1.getState().getAngularVelocity())
                        .voltage(wheel1.getState().getVoltage()),
                this,
                "wheel1-sysIdRoutine");
        SysIdRoutine wheel0Routine = new SysIdRoutine(wheel0Config, wheel0Mechanism);
        SysIdRoutine wheel1Routine = new SysIdRoutine(wheel1Config, wheel1Mechanism);
        SmartDashboard.putData("Wheel0 Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(wheel0Routine.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Wheel0 Quasistatic Forward"));
        SmartDashboard.putData("Wheel0 Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(wheel0Routine.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Wheel0 Quasistatic Reverse"));
        SmartDashboard.putData("Wheel0 Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(wheel0Routine.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Wheel0 Dynamic Forward"));
        SmartDashboard.putData("Wheel0 Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(wheel0Routine.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Wheel0 Dynamic Reverse"));
        SmartDashboard.putData("Wheel1 Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(wheel1Routine.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Wheel1 Quasistatic Forward"));
        SmartDashboard.putData("Wheel1 Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(wheel1Routine.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Wheel1 Quasistatic Reverse"));
        SmartDashboard.putData("Wheel1 Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(wheel1Routine.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Wheel1 Dynamic Forward"));
        SmartDashboard.putData("Wheel1 Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(wheel1Routine.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Wheel1 Dynamic Reverse"));

        if (Utils.isSimulation()) {
            startWheel0SimThread();
            startWheel1SimThread();
        }
    }

    public Command applyRequest(Supplier<Pair<IntakeWheelRequest, IntakeWheelRequest>> requestSupplier) {
        return run(() -> {
            wheel0.setControl(requestSupplier.get().getFirst());
            wheel1.setControl(requestSupplier.get().getSecond());
        });
    }

    Command idle() {
        IntakeWheelRequest.VoltageRequest request0 = new IntakeWheelRequest.VoltageRequest().withVoltage(Volts.of(-1.0));
        IntakeWheelRequest.VoltageRequest request1 = new IntakeWheelRequest.VoltageRequest().withVoltage(Volts.of(3.0));
        Pair<IntakeWheelRequest, IntakeWheelRequest> request = new Pair<>(request0, request1);
        return applyRequest(() -> request)
                .withName(String.format("%s: IDLE", getName()));
    }

    @Override
    public void periodic() {
        wheel0.update();
        wheel1.update();
        objectDetector.update();
    }

    private void startWheel0SimThread() {
        wheel0LastSimTime = Utils.getCurrentTimeSeconds();

        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - wheel0LastSimTime;
            wheel0LastSimTime = currentTime;

            wheel0.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(wheel0SimLoopPeriod.baseUnitMagnitude());
    }

    private void startWheel1SimThread() {
        wheel1LastSimTime = Utils.getCurrentTimeSeconds();

        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - wheel1LastSimTime;
            wheel1LastSimTime = currentTime;

            wheel1.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(wheel1SimLoopPeriod.baseUnitMagnitude());
    }
}
