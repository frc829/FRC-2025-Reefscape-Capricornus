package digilib.cameraPower;

import au.grapplerobotics.MitoCANdria;
import edu.wpi.first.wpilibj.Alert;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MitocandriaPowerModule implements CameraPower {

    private final CameraPowerState state;
    private CameraPowerRequest request;
    private final CameraPowerTelemetry telemetry;
    private final MitoCANdria mitoCANdria;
    private final Map<String, Double> voltages = new HashMap<>();
    private final Map<String, Double> currents = new HashMap<>();
    private final Alert alert = new Alert("Error", Alert.AlertType.kError);


    public MitocandriaPowerModule(CameraPowerConstants constants, MitoCANdria mitoCANdria) {
        telemetry = new CameraPowerTelemetry(constants.name());
        this.mitoCANdria = mitoCANdria;
        state = new CameraPowerState(List.of(
                "MITOCANDRIA_CHANNEL_5VA",
                "MITOCANDRIA_CHANNEL_5VB",
                "MITOCANDRIA_CHANNEL_ADJ",
                "MITOCANDRIA_CHANNEL_USB1",
                "MITOCANDRIA_CHANNEL_USB2"));
    }

    @Override
    public CameraPowerState getState() {
        return state;
    }

    @Override
    public void setControl(CameraPowerRequest request) {
        if (this.request != request) {
            this.request = request;
        }
        request.apply(this);
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        try {
            currents.put("MITOCANDRIA_CHANNEL_5VA", mitoCANdria.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_5VA).orElse(Double.NaN));
            currents.put("MITOCANDRIA_CHANNEL_5VB", mitoCANdria.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_5VB).orElse(Double.NaN));
            currents.put("MITOCANDRIA_CHANNEL_ADJ", mitoCANdria.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ).orElse(Double.NaN));
            currents.put("MITOCANDRIA_CHANNEL_USB1", mitoCANdria.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_USB1).orElse(Double.NaN));
            currents.put("MITOCANDRIA_CHANNEL_USB2", mitoCANdria.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_USB2).orElse(Double.NaN));

            voltages.put("MITOCANDRIA_CHANNEL_5VA", mitoCANdria.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_5VA).orElse(Double.NaN));
            voltages.put("MITOCANDRIA_CHANNEL_5VB", mitoCANdria.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_5VB).orElse(Double.NaN));
            voltages.put("MITOCANDRIA_CHANNEL_ADJ", mitoCANdria.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ).orElse(Double.NaN));
            voltages.put("MITOCANDRIA_CHANNEL_USB1", mitoCANdria.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_USB1).orElse(Double.NaN));
            voltages.put("MITOCANDRIA_CHANNEL_USB2", mitoCANdria.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_USB2).orElse(Double.NaN));
            alert.set(false);
        } catch (Exception e) {
            alert.setText("Error: " + e.getMessage());
            alert.set(true);
        }
        state.setVoltages(voltages);
        state.setCurrents(currents);
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }
}
