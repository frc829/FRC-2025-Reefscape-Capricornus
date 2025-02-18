package frc.robot.subsystems.arm;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.arm.Arm;
import digilib.arm.ArmRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ArmSubsystem implements Subsystem {
    private final Arm arm;
    private double lastSimTime;
    private final Time simLoopPeriod;

    public ArmSubsystem(Arm arm, Time simLoopPeriod) {
        this.arm = arm;
        this.simLoopPeriod = simLoopPeriod;

        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.per(Second).of(1.0),
                Volts.of(3.0),
                Seconds.of(10.0),
                state -> SignalLogger.writeString("arm-sysIdRoutine", state.toString()));
        ArmRequest.VoltageRequest voltageRequest = new ArmRequest.VoltageRequest();
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                volts -> arm.setControl(voltageRequest.withVoltage(volts)),
                null,
                this);
        SysIdRoutine routine = new SysIdRoutine(config, mechanism);
        SmartDashboard.putData("Arm Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Arm Quasistatic Forward"));
        SmartDashboard.putData("Arm Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Arm Quasistatic Reverse"));
        SmartDashboard.putData("Arm Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Arm Dynamic Forward"));
        SmartDashboard.putData("Arm Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Arm Dynamic Reverse"));

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Trigger atPosition(Angle position, Angle tolerance) {
        return new Trigger(() -> arm.getState().getAngle().isNear(position, tolerance));
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
