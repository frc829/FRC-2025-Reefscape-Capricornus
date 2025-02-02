package frc.robot.subsystems;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
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
    private final MutTime currentTime = Seconds.mutable(Timer.getFPGATimestamp());
    private final MutTime lastSimTime = currentTime.mutableCopy();
    private final MutVoltage supplyVoltage = Volts.mutable(0.0);
    private final MutTime deltaTime = Seconds.mutable(0.0);



    public CommandArm(Arm arm, Time simLoopPeriod) {
        this.arm = arm;
    }

    public Command applyRequest(Supplier<ArmRequest> requestSupplier) {
        return run(() -> arm.setControl(requestSupplier.get())).handleInterrupt(arm::disableHold);
    }

    @Override
    public void periodic() {
        arm.update();
        ArmState armState = arm.getState();
    }

    public void startSimThread() {
        currentTime.mut_setMagnitude(Timer.getFPGATimestamp());
        deltaTime.mut_setMagnitude(currentTime.baseUnitMagnitude() - lastSimTime.baseUnitMagnitude());
        lastSimTime.mut_replace(currentTime);
        supplyVoltage.mut_setMagnitude(12.0);
        arm.updateSimState(deltaTime, supplyVoltage);
    }
}



