package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.elevator.Elevator;
import digilib.elevator.ElevatorRequest;

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
