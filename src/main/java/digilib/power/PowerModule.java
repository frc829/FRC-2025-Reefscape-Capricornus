package digilib.power;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;

public class PowerModule implements Power {

    private final PowerState state;
    private PowerRequest request;
    // private final PowerTelemetry telemetry;
    private final PowerDistribution pdh;

    public PowerModule(PowerConstants constants) {
        pdh = new PowerDistribution(constants.module(), constants.moduleType());
        state = new PowerState(pdh.getNumChannels());
        // telemetry = new PowerTelemetry(
        //         constants.name(),
        //         pdh);
    }

    @Override
    public PowerState getState() {
        return state;
    }

    @Override
    public void setControl(PowerRequest request) {
        if (this.request != request) {
            this.request = request;
        }
        request.apply(this);
    }

    @Override
    public void clearStickyFaults() {
        pdh.clearStickyFaults();
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    private void updateState() {
        state.withCurrents()
                .withVoltage(pdh.getVoltage())
                .withTemperature(pdh.getTemperature())
                .withCurrents(pdh.getAllCurrents())
                .withTotalCurrent(pdh.getTotalCurrent())
                .withTotalPower(pdh.getTotalPower())
                .withTotalEnergy(pdh.getTotalPower())
                .withTimeStamp(Timer.getFPGATimestamp());
    }

    @Override
    public void updateTelemetry() {
        // telemetry.telemeterize(state);
    }
}
