package digilib.claws;

import digilib.SolenoidType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class SolenoidClaw implements Claw {
    private final ClawState clawState = new ClawState();
    private final PneumaticsModuleType moduleType;
    private final ClawValue solenoidOnClawValue;
    private ClawRequest request;
    private final ClawTelemetry telemetry;
    private final Solenoid solenoid;

    public SolenoidClaw(
            ClawConstants clawConstants,
            Solenoid solenoid) {
        this.solenoid = solenoid;
        this.solenoidOnClawValue = clawConstants.solenoidOnClawValue();
        this.moduleType = clawConstants.moduleType();
        this.telemetry = new ClawTelemetry(
                clawConstants.name(),
                clawConstants.solenoidOnClawValue());
    }

    @Override
    public SolenoidType getSolenoidType() {
        return SolenoidType.SINGLE;
    }

    @Override
    public PneumaticsModuleType getPneumaticsModuleType() {
        return moduleType;
    }

    @Override
    public ClawState getState() {
        return clawState;
    }

    @Override
    public void setControl(ClawRequest request) {
        if (this.request != request) {
            this.request = request;
        }
        request.apply(this);
    }

    @Override
    public void setValue(ClawValue clawValue) {
        solenoid.set(clawValue == solenoidOnClawValue);
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        clawState.withClawValue(solenoid.get() ? solenoidOnClawValue : solenoidOnClawValue.opposite())
                .withTimestamp(Timer.getFPGATimestamp());
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(clawState);
    }
}
