package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.Utils;
import digilib.elevator.Elevator;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class ElevatorSubsystem implements Subsystem {
    private final Elevator elevator;
    private double lastSimTime;
    private final Time simLoopPeriod;

    public ElevatorSubsystem(Elevator elevator, Time simLoopPeriod) {
        this.elevator = elevator;
        this.simLoopPeriod = simLoopPeriod;
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command toHeight(double meters) {
        return run(() -> elevator.setPosition(meters))
                .withName(String.format("%s: %.2f meters", getName(), meters));
    }

    public Command toVelocity(DoubleSupplier scalarSetpoint) {
        return run(() -> elevator.setVelocity(scalarSetpoint.getAsDouble()))
                .withName(String.format("%s: VELOCITY", getName()));
    }

    public Command toVoltage(DoubleSupplier volts) {
        return run(() -> elevator.setVoltage(volts.getAsDouble()))
                .withName(String.format("%s: VOLTAGE Dangerous", getName()));
    }

    public Command hold() {
        MutDistance holdPosition = Meters.mutable(0.0);
        return sequence(
                runOnce(() -> holdPosition.mut_setMagnitude(elevator.getState().getMotorEncoderPositionMeters())),
                run(() -> elevator.setPosition(holdPosition.in(Meters))))
                .withName(String.format("%s: HOLD", getName()));
    }

    @Override
    public void periodic() {
        elevator.update();
    }

    @SuppressWarnings("resource")
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            elevator.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        }).startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
