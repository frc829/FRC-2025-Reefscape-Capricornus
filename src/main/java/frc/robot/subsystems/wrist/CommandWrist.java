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

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class CommandWrist implements Subsystem {
    private final Wrist wrist;
    private double lastSimTime;
    private final Time simLoopPeriod;

    public CommandWrist(Wrist wrist, Time simLoopPeriod) {
        this.wrist = wrist;
        this.simLoopPeriod = simLoopPeriod;

        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.per(Second).of(1.0),
                Volts.of(3.0),
                Seconds.of(10.0));
        WristRequest.VoltageRequest voltageRequest = new WristRequest.VoltageRequest();
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                volts -> wrist.setControl(voltageRequest.withVoltage(volts)),
                log -> log
                        .motor("wrist")
                        .angularVelocity(wrist.getState().getVelocity())
                        .angularPosition(wrist.getState().getPosition())
                        .voltage(wrist.getState().getVoltage()),
                this,
                "wrist-sysIdRoutine");
        SysIdRoutine routine = new SysIdRoutine(config, mechanism);
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

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Trigger atPosition(Angle position, Angle tolerance) {
        return new Trigger(() -> wrist.getState().getPosition().isNear(position, tolerance));
    }

    public Command applyRequest(Supplier<WristRequest> requestSupplier) {
        return run(() -> wrist.setControl(requestSupplier.get()));
    }

    public Command hold() {
        WristRequest.Position request = new WristRequest.Position();
        return Commands.runOnce(() -> request.withPosition(wrist.getState().getPosition().in(Radians)))
                .andThen(applyRequest(() -> request)).withName("WRIST:HOLD");
    }

    public Command goToAngle(Angle position) {
        WristRequest.Position request = new WristRequest.Position();
        request.withPosition(position.in(Radians));
        return applyRequest(() -> request)
                .withName(String.format("WRIST:%s degrees", position.in(Degrees)));
    }

    public Command moveAtVelocity(DoubleSupplier value) {
        WristRequest.Velocity request = new WristRequest.Velocity();
        return applyRequest(() -> request.withVelocity(value.getAsDouble())).withName("WRIST:VELOCITY");
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
