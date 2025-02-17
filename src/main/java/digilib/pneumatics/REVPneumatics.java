package digilib.pneumatics;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;

public class REVPneumatics implements Pneumatics {
    private final PneumaticsState state = new PneumaticsState();
    private PneumaticsRequest request;
    // private final PneumaticsTelemetry telemetry;
    private final PneumaticHub pneumaticHub;

    public REVPneumatics(
            PneumaticsConstants constants,
            PneumaticHub pneumaticHub) {
        this.pneumaticHub = pneumaticHub;
        // this.telemetry = new PneumaticsTelemetry(constants.name(), pneumaticHub);
    }

    @Override
    public PneumaticsState getState() {
        return state;
    }

    @Override
    public void setControl(PneumaticsRequest request) {
        if (this.request != request) {
            this.request = request;
        }
        request.apply(this);
    }

    @Override
    public void clearStickyFaults() {
        pneumaticHub.clearStickyFaults();
    }

    @Override
    public void turnOnCompressor() {
        pneumaticHub.enableCompressorDigital();
    }

    @Override
    public void turnOffCompressor() {
        pneumaticHub.disableCompressor();
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.withCompressorOn(pneumaticHub.getCompressor())
                .withTimestamp(Timer.getFPGATimestamp());
    }

    @Override
    public void updateTelemetry() {
        // telemetry.telemeterize(state);
    }
}
