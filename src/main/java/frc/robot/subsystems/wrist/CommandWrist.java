package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.wrist.Wrist;
import digilib.wrist.WristRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

public class CommandWrist implements Subsystem {
    private final Time simLoopPeriod;
    private final Wrist wrist;
    private double lastSimTime;

    public CommandWrist(Wrist wrist, Time simLoopPeriod) {
        this.wrist = wrist;
        this.simLoopPeriod = simLoopPeriod;
        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    public Trigger atPosition(Angle position, Angle tolerance) {
        return new Trigger(() -> wrist.getState().getPosition().isNear(position, tolerance));
    }


    public Command applyRequest(Supplier<WristRequest> requestSupplier) {
        return run(() -> wrist.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        wrist.update();
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
            wrist.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }


}
