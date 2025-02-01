package frc.robot.subsystems;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.elevator.Elevator;
import frc.robot.mechanisms.elevator.ElevatorRequest;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class CommandElevator implements Subsystem {
    private final Elevator elevator;
    private final Time simLoopPeriod;

    public CommandElevator(Elevator elevator, Time simLoopPeriod) {
        this.elevator = elevator;
        this.simLoopPeriod = simLoopPeriod;
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
        final MutTime currentTime = Seconds.mutable(Timer.getFPGATimestamp());
        final MutTime lastSimTime = currentTime.mutableCopy();
        final MutVoltage supplyVoltage = Volts.mutable(0.0);
        MutTime deltaTime = Seconds.mutable(0.0);
        try (Notifier simNotifier = new Notifier(() -> {
            currentTime.mut_setMagnitude(Timer.getFPGATimestamp());
            deltaTime.mut_setMagnitude(currentTime.baseUnitMagnitude() - lastSimTime.baseUnitMagnitude());
            lastSimTime.mut_replace(currentTime);
            supplyVoltage.mut_setMagnitude(RobotController.getBatteryVoltage());
            elevator.updateSimState(deltaTime, supplyVoltage);
        })) {
            simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
        }
    }


}
