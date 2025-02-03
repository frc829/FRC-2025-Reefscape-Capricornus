package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.intake.Intake;
import frc.robot.mechanisms.intake.IntakeMotorRequest;

import java.util.function.Supplier;

public class CommandIntake implements Subsystem {
    private static final double simLoopPeriod = 0.005;
    private final Intake intake;
    private double lastSimTime;

    public CommandIntake(Intake intake) {
        this.intake = intake;
        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<IntakeMotorRequest> requestSupplier) {
        return run(() -> intake.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        intake.update();
    }

    private void startSimThread() {
        lastSimTime = Timer.getFPGATimestamp();
        try (Notifier simNotifier = new Notifier(() -> {
            final double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            intake.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        })) {
            simNotifier.startPeriodic(simLoopPeriod);
        }
    }


}
