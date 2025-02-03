package frc.robot.mechanisms.claws;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import java.util.Map;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.mechanisms.claws.ClawState.*;

public class SolenoidClaw implements Claw {
    private final ClawState lastClawState = new ClawState();
    private final ClawState clawState = new ClawState();
    private final Solenoid solenoid;
    private final Map<Boolean, ClawValue> solenoidClawValueMap;
    private ClawRequest clawRequest;
    private final Map<ClawValue, Boolean> clawValueSolenoidMap;
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final ClawTelemetry clawTelemetry;

    public SolenoidClaw(
            ClawConstants clawConstants,
            Solenoid solenoid) {
        this.solenoid = solenoid;
        clawValueSolenoidMap = clawConstants.getClawValueSolenoidMap();
        solenoidClawValueMap = clawConstants.getSolenoidClawValueMap();
        this.clawTelemetry = new ClawTelemetry(clawConstants.getName());
    }

    @Override
    public void setControl(ClawRequest request) {
        if(clawRequest != request){
            clawRequest = request;
        }
        request.apply(this);
    }

    @Override
    public ClawState getState() {
        return clawState;
    }

    @Override
    public ClawState getStateCopy() {
        return clawState.clone();
    }

    @Override
    public ClawState getLastState() {
        return lastClawState;
    }

    @Override
    public void updateTelemetry() {
        clawTelemetry.telemeterize(clawState);
    }

    @Override
    public void setValue(ClawValue clawValue) {
        solenoid.set(clawValueSolenoidMap.get(clawValue));
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    private void updateState(){
        lastClawState.withClawState(clawState);
        ClawValue clawValue = solenoidClawValueMap.get(solenoid.get());
        clawState.withClawValue(clawValue);
        clawState.withTimestamp(timestamp.mut_setMagnitude(Timer.getFPGATimestamp()));
    }
}
