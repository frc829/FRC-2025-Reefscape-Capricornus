package frc.robot.subsystems.intakeWheel;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.intakeWheel.IntakeWheel;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;
import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;

public class IntakeWheelSubsystem implements Subsystem {
    private final IntakeWheel intakeWheel;
    private double lastSimTime;
    private final Time simLoopPeriod;

    public IntakeWheelSubsystem(
            IntakeWheel intakeWheel,
            Time simLoopPeriod) {
        this.intakeWheel = intakeWheel;
        this.simLoopPeriod = simLoopPeriod;
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Trigger gte(double currentAmps) {
        return new Trigger(() -> intakeWheel
                .getState()
                .getAmps() >= currentAmps);
    }

    public Trigger lte(double currentAmps) {
        return new Trigger(() -> intakeWheel
                .getState()
                .getAmps() <= currentAmps);
    }

    public Trigger inRange(double minCurrentAmps, double maxCurrentAmps) {
        return gte(minCurrentAmps).and(lte(maxCurrentAmps));
    }

    public Command toVelocity(DoubleSupplier scalarSetpoint) {
        return run(() -> intakeWheel.setVelocity(scalarSetpoint.getAsDouble()))
                .withName(String.format("%s: VELOCITY", getName()));
    }

    public Command toVelocityWithCondition(DoubleSupplier scalarSetpoint, BooleanSupplier condition) {
        return run(() -> {
            if (condition.getAsBoolean()) {
                intakeWheel.setVelocity(0.0);
            }else{
                intakeWheel.setVelocity(scalarSetpoint.getAsDouble());
        }
        }).withName(String.format("%s: VELOCITY WITH CONDITION", getName()));
    }

    public Command toVoltage(double volts) {
        return run(() -> intakeWheel.setVoltage(volts))
                .withName(String.format("%s: VOLTAGE", getName()));
    }

    @Override
    public void periodic() {
        intakeWheel.update();
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            intakeWheel.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
