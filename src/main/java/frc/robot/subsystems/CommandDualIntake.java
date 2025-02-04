package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.mechanisms.intakeWheel.IntakeWheel;
import frc.robot.mechanisms.intakeWheel.IntakeWheelRequest;

import java.util.function.Supplier;

public class CommandDualIntake implements Subsystem {
    private final IntakeWheel wheel0;
    private final IntakeWheel wheel1;
    private double lastSimTime;
    public final Trigger hasCoral;
    private final Time simLoopPeriod;

    public CommandDualIntake(IntakeWheel wheel0, IntakeWheel wheel1, Trigger hasCoral, Time simLoopPeriod) {
        this.wheel0 = wheel0;
        this.wheel1 = wheel1;
        this.hasCoral = hasCoral;
        this.simLoopPeriod = simLoopPeriod;
        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<Pair<IntakeWheelRequest, IntakeWheelRequest>> requestSupplier) {
        return run(() -> {
            wheel0.setControl(requestSupplier.get().getFirst());
            wheel1.setControl(requestSupplier.get().getSecond());
        });
    }

    @Override
    public void periodic() {
        wheel0.update();
        wheel1.update();
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
            wheel0.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            wheel1.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }


}
