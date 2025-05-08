package digilib.motorControllers.ctre;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import digilib.motorControllers.MotorController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class TalonFXController implements MotorController {
    private final TalonFX talonFX;
    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
    private final MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0.0);
    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0);
    private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityTorqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0.0);

    public TalonFXController(TalonFX talonFX) {
        this.talonFX = talonFX;
    }

    public TalonFXController(TalonFX talonFX, LinearSystem<N2, N1, N2> sim){
        this(talonFX);
    }

    @Override
    public double getVoltage() {
        return talonFX.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getCurrentAmps() {
        return talonFX.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public double getPosition() {
        return talonFX.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return talonFX.getVelocity().getValueAsDouble();
    }

    @Override
    public double getAcceleration() {
        return talonFX.getAcceleration().getValueAsDouble();
    }

    @Override
    public void applyVoltage(double voltage) {
        talonFX.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void applyCurrentAmps(double currentAmps) {
        talonFX.setControl(torqueCurrentFOC.withOutput(currentAmps));
    }

    @Override
    public void applyPositionWithVoltage(double position) {
        talonFX.setControl(motionMagicExpoVoltage.withPosition(position));
    }

    @Override
    public void applyPositionWithCurrent(double position) {
        talonFX.setControl(motionMagicExpoTorqueCurrentFOC.withPosition(position));
    }

    @Override
    public void applyVelocityWithVoltage(double velocity) {
        talonFX.setControl(motionMagicVelocityVoltage.withVelocity(velocity));
    }

    @Override
    public void applyVelocityWithCurrent(double velocity) {
        talonFX.setControl(motionMagicVelocityTorqueCurrentFOC.withVelocity(velocity));
    }

    private static class SimThread{
        private static final double simLoopPeriodSeconds = 0.001;
        private final Notifier notifier;
        private double lastTimeSeconds = 0.0;

        public SimThread(
                TalonFXSimState talonFXSimState,
                LinearSystemSim<N2, N1, N2> sim,
                double sensorToMechanismRatio){
            notifier = new Notifier(() -> {
                final double currentTimeSeconds = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTimeSeconds - lastTimeSeconds;
                double inputVoltage = talonFXSimState.getMotorVoltage();
                sim.setInput(inputVoltage);
                sim.update(deltaTime);
                talonFXSimState.setRawRotorPosition(sim.getOutput(0) * sensorToMechanismRatio );
                talonFXSimState.setRotorVelocity(sim.getOutput(1) * sensorToMechanismRatio);
                talonFXSimState.setRotorAcceleration();
            });
        }
    }


}
