package frc.robot.subsystems.winch;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.winch.Winch;
import digilib.winch.WinchRequest;

import java.util.function.Supplier;

public class CommandWinch implements Subsystem {
    private final Time simLoopPeriod;
    private final Winch winch;
    private double lastSimTime;

    public CommandWinch(Winch winch, Time simLoopPeriod) {
        this.winch = winch;
        this.simLoopPeriod = simLoopPeriod;
        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<WinchRequest> requestSupplier) {
        return run(() -> winch.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        winch.update();
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
            winch.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }


}
