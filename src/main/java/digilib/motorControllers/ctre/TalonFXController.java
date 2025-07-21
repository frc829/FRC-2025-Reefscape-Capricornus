package digilib.motorControllers.ctre;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import digilib.motorControllers.MotorController;

public class TalonFXController implements MotorController {

    private final TalonFX talonFX;
    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC;
    private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityTorqueCurrentFOC;
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);

    public TalonFXController(TalonFX talonFX,
                             MotionMagicExpoVoltage motionMagicExpoVoltage,
                             MotionMagicVelocityVoltage motionMagicVelocityVoltage,
                             MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC,
                             MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityTorqueCurrentFOC) {
        this.talonFX = talonFX;
        this.motionMagicExpoVoltage = motionMagicExpoVoltage;
        this.motionMagicVelocityVoltage = motionMagicVelocityVoltage;
        this.motionMagicExpoTorqueCurrentFOC = motionMagicExpoTorqueCurrentFOC;
        this.motionMagicVelocityTorqueCurrentFOC = motionMagicVelocityTorqueCurrentFOC;
    }

    @Override
    public double getVoltageVolts() {
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
    public void applyVoltage(double voltageVolts) {
        talonFX.setControl(voltageOut.withOutput(voltageVolts));
    }

    @Override
    public void applyCurrent(double currentAmps) {
        talonFX.setControl(torqueCurrentFOC.withOutput(currentAmps));
    }

    @Override
    public void applyPositionUsingVoltage(double position) {
        talonFX.setControl(motionMagicExpoVoltage.withPosition(position));
    }

    @Override
    public void applyPositionUsingCurrent(double position) {
        talonFX.setControl(motionMagicExpoTorqueCurrentFOC.withPosition(position));
    }

    @Override
    public void applyVelocityUsingVoltage(double velocity) {
        talonFX.setControl(motionMagicVelocityVoltage.withVelocity(velocity));
    }

    @Override
    public void applyVelocityUsingCurrent(double velocity) {
        talonFX.setControl(motionMagicVelocityTorqueCurrentFOC.withVelocity(velocity));
    }
}
