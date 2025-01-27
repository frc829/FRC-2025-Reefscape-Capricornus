package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.elevator.Elevator;
import frc.robot.mechanisms.elevator.ElevatorRequest;

import java.util.function.Supplier;

public class CommandElevator implements Subsystem {
    private static final double simLoopPeriod = 0.005;
    private final Elevator elevator;
    private double lastSimTime;

    public CommandElevator(Elevator elevator) {
        this.elevator = elevator;
        if (RobotBase.isSimulation()) {
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
        lastSimTime = Timer.getFPGATimestamp();
        try (Notifier simNotifier = new Notifier(() -> {
            final double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            elevator.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        })) {
            simNotifier.startPeriodic(simLoopPeriod);
        }
    }


}
