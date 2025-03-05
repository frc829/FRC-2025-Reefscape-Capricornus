package frc.robot.subsystems.arm;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.arm.Arm;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

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
        return new Trigger (() -> arm
                .getState()
                .getMotorEncoderPositionDegrees() >= angleDegrees);
    }

    public Trigger lte(double angleDegrees) {
        return new Trigger (() -> arm
                .getState()
                .getMotorEncoderPositionDegrees() <= angleDegrees);
    }

    public Trigger inRange(double minAngleDegrees, double maxAngleDegrees) {
        return gte(maxAngleDegrees).and(lte(minAngleDegrees));
    }



    public Command applyRequest(Supplier<ArmRequest> requestSupplier) {
        return run(() -> arm.setControl(requestSupplier.get()));
    }

    Command hold() {
         ArmRequest.Position request = new ArmRequest.Position();
         return Commands.runOnce(() -> request.withPosition(arm.getState().getAngle()))
                 .andThen(applyRequest(() -> request))
                 .withName(String.format("%s: HOLD", getName()));
    }

    @Override
    public void periodic() {
        arm.update();
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            arm.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }


}
