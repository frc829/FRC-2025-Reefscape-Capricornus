package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.elevator.Elevator;
import digilib.elevator.ElevatorRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

public class CommandElevator implements Subsystem {
    private final Elevator elevator;
    private double lastSimTime;
    private final Time simLoopPeriod;

    public CommandElevator(Elevator elevator, Time simLoopPeriod) {
        this.elevator = elevator;
        this.simLoopPeriod = simLoopPeriod;
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Trigger atPosition(Distance position, Distance tolerance) {
        // TODO: return a new Trigger passing in a lambda expression
        // (cont.) The lambda is a BooleanSupplier of the form
        // (cont.) () ->
        // (cont.) on the right side of the arrow you will
        // (cont.) call elevator.getState()'s getPosition()'s isNear() method
        // (cont.) so with dots
        // (cont.) passing in position and tolerance.
        return null; // TODO: remove this when done.
    }

    public Command applyRequest(Supplier<ElevatorRequest> requestSupplier) {
        return run(() -> elevator.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        elevator.update();
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            elevator.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
