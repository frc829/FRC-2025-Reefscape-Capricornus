package frc.robot.subsystems.climber;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.climber.Climber;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    public Trigger gte(double lengthMeters) {
        return new Trigger(() -> climber
                .getState()
                .getMotorEncoderPositionMeters() >= lengthMeters);
    }

    public Trigger lte(double lengthMeters) {
        return new Trigger(() -> climber
                .getState()
                .getMotorEncoderPositionMeters() <= lengthMeters);
    }

    public Trigger inRange(double minLengthMeters, double maxLengthMeters) {
        return gte(minLengthMeters).and(lte(maxLengthMeters));
    }

    public Command toVoltage(double volts) {
        return run(() -> climber.setVoltage(volts))
                .withName(String.format("%s: VOLTAGE", getName()));
    }

    @Override
    public void periodic() {
        climber.update();
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
            climber.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
