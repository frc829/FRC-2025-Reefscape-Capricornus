package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.wrist.Wrist;
import frc.robot.mechanisms.wrist.WristRequest;

import java.util.function.Supplier;

public class CommandWrist implements Subsystem {
    private static final double simLoopPeriod = 0.005;
    private final Wrist wrist;
    private double lastSimTime;

    public CommandWrist(Wrist wrist) {
        this.wrist = wrist;
        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<WristRequest> requestSupplier) {
        return run(() -> wrist.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        wrist.update();
    }

    private void startSimThread() {
        lastSimTime = Timer.getFPGATimestamp();
        try (Notifier simNotifier = new Notifier(() -> {
            final double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            wrist.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        })) {
            simNotifier.startPeriodic(simLoopPeriod);
        }
    }


}
