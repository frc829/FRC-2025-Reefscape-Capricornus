package frc.robot.subsystems.dualIntake;

import com.ctre.phoenix6.Utils;
import digilib.objectDetectors.ObjectDetector;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import digilib.intakeWheel.IntakeWheel;
import digilib.intakeWheel.IntakeWheelRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;

import static digilib.intakeWheel.IntakeWheelRequest.*;
import static edu.wpi.first.units.Units.*;

public class CommandDualIntake implements Subsystem {
    private final IntakeWheel wheel0;
    private final IntakeWheel wheel1;
    private double lastSimTime;
    private final ObjectDetector objectDetector;
    private final Time simLoopPeriod;
    public final Trigger hasCoral;

    public CommandDualIntake(IntakeWheel wheel0, IntakeWheel wheel1, ObjectDetector objectDetector, Time simLoopPeriod) {

        this.wheel0 = wheel0;
        this.wheel1 = wheel1;
        this.objectDetector = objectDetector;
        this.simLoopPeriod = simLoopPeriod;
        hasCoral = new Trigger(objectDetector.getState()::isInRange);

        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.per(Second).of(1.0),
                Volts.of(7.0),
                Seconds.of(10.0));
        VoltageRequest intakeWheel0VoltageRequest = new VoltageRequest();
        VoltageRequest intakeWheel1VoltageRequest = new VoltageRequest();
        SysIdRoutine.Mechanism wheel0Mechanism = new SysIdRoutine.Mechanism(
                volts -> wheel0.setControl(intakeWheel0VoltageRequest.withVoltage(volts)),
                log -> log
                        .motor("wheel0")
                        .angularVelocity(wheel0.getState().getAngularVelocity())
                        .voltage(wheel0.getState().getVoltage()),
                this,
                "wheel0-sysIdRoutine");
        SysIdRoutine.Mechanism wheel1Mechanism = new SysIdRoutine.Mechanism(
                volts -> wheel0.setControl(intakeWheel1VoltageRequest.withVoltage(volts)),
                log -> log
                        .motor("wheel1")
                        .angularVelocity(wheel1.getState().getAngularVelocity())
                        .voltage(wheel1.getState().getVoltage()),
                this,
                "wheel1-sysIdRoutine");
        SysIdRoutine wheel0Routine = new SysIdRoutine(config, wheel0Mechanism);
        SysIdRoutine wheel1Routine = new SysIdRoutine(config, wheel1Mechanism);
        SmartDashboard.putData("Wheel0 Quasi Forward", wheel0Routine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Wheel0 Quasi Reverse", wheel0Routine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Wheel0 Dynam Forward", wheel0Routine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Wheel0 Dynam Reverse", wheel0Routine.dynamic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Wheel1 Quasi Forward", wheel1Routine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Wheel1 Quasi Reverse", wheel1Routine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Wheel1 Dynam Forward", wheel1Routine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Wheel1 Dynam Reverse", wheel1Routine.dynamic(SysIdRoutine.Direction.kReverse));

        if (RobotBase.isSimulation()) {
            startSimThread();
        }

    }

    public Command applyRequest(Supplier<Pair<IntakeWheelRequest, IntakeWheelRequest>> requestSupplier) {
        return run(() -> {
            wheel0.setControl(requestSupplier.get().getFirst());
            wheel1.setControl(requestSupplier.get().getSecond());
        });
    }

    @Override
    public void periodic() {
        wheel0.update();
        wheel1.update();
        objectDetector.update();
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
            wheel0.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            wheel1.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }


}
