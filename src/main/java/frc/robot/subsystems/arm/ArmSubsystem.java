package frc.robot.subsystems.arm;

import com.ctre.phoenix6.Utils;
import digilib.arm.Arm;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class ArmSubsystem implements Subsystem {
    private final Arm arm;
    private double lastSimTime;
    private final Time simLoopPeriod;

    public ArmSubsystem(Arm arm, Time simLoopPeriod) {
        this.arm = arm;
        this.simLoopPeriod = simLoopPeriod;
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Trigger gte(double angleDegrees) {
        return new Trigger(() -> arm
                .getMotorEncoderPositionDegrees() >= angleDegrees);
    }

    public Trigger lte(double angleDegrees) {
        return new Trigger(() -> arm
                .getMotorEncoderPositionDegrees() <= angleDegrees);
    }

    public Command toAngle(double degrees) {
        return run(() -> arm.applyPosition(degrees / 360.0))
                .withName(String.format("%s: %.2f deg", getName(), degrees));
    }

    public Command toVelocity(DoubleSupplier scalarSetpoint) {
        return run(() -> arm.applyVelocity(scalarSetpoint.getAsDouble()))
                .withName(String.format("%s: VELOCITY", getName()));
    }

    public Command hold() {
        MutAngle holdPosition = Rotations.mutable(0.0);
        return sequence(
                runOnce(() -> holdPosition.mut_setMagnitude(arm.getAbsoluteEncoderPositionRotations())),
                run(() -> arm.applyPosition(holdPosition.in(Rotations))))
                .withName(String.format("%s: HOLD", getName()));
    }

    @Override
    public void periodic() {
        arm.update();
    }

    @SuppressWarnings("resource")
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            arm.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        }).startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
