package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.elevator.Elevator;
import digilib.elevator.ElevatorRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem implements Subsystem {
    private final Elevator elevator;
    private double lastSimTime;
    private final Time simLoopPeriod;

    public ElevatorSubsystem(Elevator elevator, Time simLoopPeriod) {
        this.elevator = elevator;
        this.simLoopPeriod = simLoopPeriod;

        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.per(Second).of(1.0),
                Volts.of(7.0),
                Seconds.of(10.0));
        ElevatorRequest.VoltageRequest voltageRequest = new ElevatorRequest.VoltageRequest();
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                volts -> elevator.setControl(voltageRequest.withVoltage(volts)),
                log -> log
                        .motor("elevator")
                        .linearVelocity(elevator.getState().getVelocity())
                        .linearPosition(elevator.getState().getHeight())
                        .voltage(elevator.getState().getVoltage()),
                this,
                "elevator-sysIdRoutine");
        SysIdRoutine routine = new SysIdRoutine(config, mechanism);
        SmartDashboard.putData("Elevator Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Elevator Quasistatic Forward"));
        SmartDashboard.putData("Elevator Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Elevator Quasistatic Reverse"));
        SmartDashboard.putData("Elevator Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Elevator Dynamic Forward"));
        SmartDashboard.putData("Elevator Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Elevator Dynamic Reverse"));

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Trigger atPosition(Distance position, Distance tolerance) {
        return new Trigger(() -> elevator.getState().getHeight().isNear(position, tolerance));
    }

    public Command applyRequest(Supplier<ElevatorRequest> requestSupplier) {
        return run(() -> elevator.setControl(requestSupplier.get()));
    }

    Command hold() {
        ElevatorRequest.Position request = new ElevatorRequest.Position();
        return Commands.runOnce(() -> request.withPosition(elevator.getState().getHeight()))
                .andThen(applyRequest(() -> request))
                .withName(String.format("%s: HOLD", getName()));
    }

    @Override
    public void periodic() {
        elevator.update();
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            SmartDashboard.putNumber("ElapsedTime", deltaTime);
            elevator.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
