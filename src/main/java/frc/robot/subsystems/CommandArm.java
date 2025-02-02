package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.mechanisms.arm.Arm;
import frc.robot.mechanisms.arm.ArmRequest;
import frc.robot.mechanisms.arm.ArmState;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class CommandArm implements Subsystem {
    private final Arm arm;
    private double lastSimTime;
    private final Time simLoopPeriod;
    private final MutTime currentTime = Seconds.mutable(Timer.getFPGATimestamp());
    private final MutVoltage supplyVoltage = Volts.mutable(0.0);
    private final MutTime deltaTime = Seconds.mutable(0.0);



    public CommandArm(Arm arm, Time simLoopPeriod) {
        this.arm = arm;
        this.simLoopPeriod = simLoopPeriod;
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<ArmRequest> requestSupplier) {
        return run(() -> arm.setControl(requestSupplier.get())).handleInterrupt(arm::disableHold);
    }

    @Override
    public void periodic() {
        arm.update();
        ArmState armState = arm.getState();
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
            arm.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}



