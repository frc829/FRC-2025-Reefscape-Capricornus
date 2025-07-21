package digilib.intake;

import com.ctre.phoenix6.Utils;
import digilib.intakeWheel.IntakeWheel;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleSupplier;

public class IntakeSubsystem implements Subsystem {
    private final IntakeWheel intakeWheel;
    private double lastSimTime;
    private final Time simLoopPeriod;

    private IntakeSubsystem(
            IntakeWheel intakeWheel,
            Time simLoopPeriod) {
        this.intakeWheel = intakeWheel;
        this.simLoopPeriod = simLoopPeriod;
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command toVelocity(DoubleSupplier scalarSetpoint) {
        return run(() -> intakeWheel.applyMotorEncoderVelocity(scalarSetpoint.getAsDouble()))
                .withName(String.format("%s: VELOCITY", getName()));
    }

    public Command toVoltage(double volts) {
        return run(() -> intakeWheel.applyVolts(volts))
                .withName(String.format("%s: VOLTAGE", getName()));
    }

    @Override
    public void periodic() {
        intakeWheel.update();
    }

    @SuppressWarnings("resource")
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            intakeWheel.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        }).startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }

    public static final IntakeSubsystem create(IntakeConfig.Spark intakeConfig){
        return null;
    }

}
