package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.arm.Arm;
import frc.robot.mechanisms.arm.ArmRequest;

import java.util.function.Supplier;

public class CommandArm implements Subsystem {
    private static final double simLoopPeriod = 0.005;
    private final Arm arm;
    private double lastSimTime;

    public CommandArm(Arm arm) {
        this.arm = arm;
        if (RobotBase.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<ArmRequest> requestSupplier) {
        return run(() -> arm.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        arm.update();
    }

    private void startSimThread() {
        lastSimTime = Timer.getFPGATimestamp();
        try (Notifier simNotifier = new Notifier(() -> {
            final double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            arm.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        })) {
            simNotifier.startPeriodic(simLoopPeriod);
        }
    }


}
