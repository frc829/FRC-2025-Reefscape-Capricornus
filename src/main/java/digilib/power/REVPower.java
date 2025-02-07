package digilib.power;

import edu.wpi.first.wpilibj.PowerDistribution;

public class REVPower implements Power {

    private final PowerDistribution pdh;
    private final PowerState lastState;
    private final PowerState state;

    public REVPower(int module) {
        pdh = new PowerDistribution(module, PowerDistribution.ModuleType.kRev);
        lastState = new PowerState(pdh.getNumChannels());
        state = new PowerState(pdh.getNumChannels());
    }

    @Override
    public PowerState getState() {
        return state;
    }

    @Override
    public PowerState getStateCopy() {
        return state.clone();
    }

    @Override
    public PowerState getLastState() {
        return lastState;
    }

    @Override
    public void setControl() {
        // TODO: will do later
    }

    @Override
    public void clearStickyFaults() {
        pdh.clearStickyFaults();
    }

    @Override
    public void update() {
        lastState.withState(state);
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
                .withFaults(pdh.getFaults())
                .withStickyFaults(pdh.getStickyFaults());
    }

    @Override
    public void updateSimState() {
        // TODO: will do later
    }

    @Override
    public void updateTelemetry() {
        // TODO: will do later
    }


}
