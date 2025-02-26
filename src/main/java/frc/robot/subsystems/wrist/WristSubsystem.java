package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.wrist.Wrist;
import digilib.wrist.WristRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class WristSubsystem implements Subsystem {
    private final Wrist wrist;
    private double lastSimTime;
    private final Time simLoopPeriod;

    public WristSubsystem(Wrist wrist, Time simLoopPeriod) {
        this.wrist = wrist;
        this.simLoopPeriod = simLoopPeriod;

        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.per(Second).of(2),
                Volts.of(2.0),
                Seconds.of(1.0));
        WristRequest.VoltageRequest voltageRequest = new WristRequest.VoltageRequest();
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                volts -> wrist.setControl(voltageRequest.withVoltage(volts)),
                log -> log
                        .motor("wrist")
                        .angularVelocity(wrist.getState().getVelocity())
                        .angularPosition(wrist.getState().getAngle())
                        .voltage(wrist.getState().getVoltage()),
                this,
                "wrist-sysIdRoutine");
        SysIdRoutine.Mechanism mechanismWithAbs = new SysIdRoutine.Mechanism(
                volts -> wrist.setControl(voltageRequest.withVoltage(volts)),
                log -> log
                        .motor("wrist")
                        .angularVelocity(wrist.getState().getAbsoluteVelocity())
                        .angularPosition(wrist.getState().getAbsolutePosition())
                        .voltage(wrist.getState().getVoltage()),
                this,
                "wrist-withAbs-sysIdRoutine");
        SysIdRoutine routine = new SysIdRoutine(config, mechanism);
        SysIdRoutine routineWithAbs = new SysIdRoutine(config, mechanismWithAbs);
        SmartDashboard.putData("Wrist Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Wrist Quasistatic Forward"));
        SmartDashboard.putData("Wrist Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Wrist Quasistatic Reverse"));
        SmartDashboard.putData("Wrist Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Wrist Dynamic Forward"));
        SmartDashboard.putData("Wrist Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Wrist Dynamic Reverse"));

        SmartDashboard.putData("Wrist Abs Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routineWithAbs.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Wrist Quasistatic Forward"));
        SmartDashboard.putData("Wrist Abs Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routineWithAbs.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Wrist Quasistatic Reverse"));
        SmartDashboard.putData("Wrist Abs Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routineWithAbs.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Wrist Dynamic Forward"));
        SmartDashboard.putData("Wrist Abs Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routineWithAbs.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Wrist Dynamic Reverse"));

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Trigger atPosition(Angle position, Angle tolerance) {
        return new Trigger(() -> wrist.getState().getAngle().isNear(position, tolerance));
    }

    public Command applyRequest(Supplier<WristRequest> requestSupplier) {
        return run(() -> wrist.setControl(requestSupplier.get()));
    }

    Command hold() {
        WristRequest.Position request = new WristRequest.Position();
        return Commands.runOnce(() -> request.withAngle(wrist.getState().getAngle()))
                .andThen(applyRequest(() -> request))
                .withName(String.format("%s: HOLD", getName()));
    }

    @Override
    public void periodic() {
        wrist.update();
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            wrist.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
