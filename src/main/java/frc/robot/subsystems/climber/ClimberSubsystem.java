package frc.robot.subsystems.climber;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.climber.Climber;

import java.util.function.DoubleSupplier;

public class ClimberSubsystem implements Subsystem {
    private final Time simLoopPeriod;
    private final Climber climber;
    private double lastSimTime;

    public ClimberSubsystem(Climber climber, Time simLoopPeriod) {
        this.climber = climber;
        this.simLoopPeriod = simLoopPeriod;

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command toVoltage(DoubleSupplier volts) {
        return run(() -> climber.applyVoltage(volts.getAsDouble()))
                .withName(String.format("%s: VOLTAGE", getName()));
    }

    @Override
    public void periodic() {
        climber.update();
    }

    @SuppressWarnings("resource")
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            climber.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        }).startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
