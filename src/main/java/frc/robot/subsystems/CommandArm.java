package frc.robot.subsystems;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.arm.Arm;
import frc.robot.mechanisms.arm.ArmRequest;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class CommandArm implements Subsystem {
    private static final double simLoopPeriod = 0.005;
    private final Arm arm;

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
        final MutTime currentTime = Seconds.mutable(Timer.getFPGATimestamp());
        final MutTime lastSimTime = currentTime.mutableCopy();
        final MutVoltage supplyVoltage = Volts.mutable(0.0);
        MutTime deltaTime = Seconds.mutable(0.0);
        try (Notifier simNotifier = new Notifier(() -> {
            currentTime.mut_setMagnitude(Timer.getFPGATimestamp());
            deltaTime.mut_setMagnitude(currentTime.baseUnitMagnitude() - lastSimTime.baseUnitMagnitude());
            lastSimTime.mut_replace(currentTime);
            supplyVoltage.mut_setMagnitude(RobotController.getBatteryVoltage());
            arm.updateSimState(deltaTime, supplyVoltage);
        })) {
            simNotifier.startPeriodic(simLoopPeriod);
        }
    }


}
