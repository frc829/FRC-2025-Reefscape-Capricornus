package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.intake.DualIntake;
import frc.robot.mechanisms.intake.DualIntakeRequest;

import java.util.function.Supplier;

public class CommandIntake implements Subsystem {
    private static final double simLoopPeriod = 0.005;
    private final DualIntake dualIntake;
    private double lastSimTime;

    public CommandIntake(DualIntake dualIntake) {
        this.dualIntake = dualIntake;
        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<DualIntakeRequest> requestSupplier) {
        return run(() -> dualIntake.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        dualIntake.update();
    }

    private void startSimThread() {
        lastSimTime = Timer.getFPGATimestamp();
        try (Notifier simNotifier = new Notifier(() -> {
            final double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            dualIntake.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        })) {
            simNotifier.startPeriodic(simLoopPeriod);
        }
    }


}
