package digilib.pneumatics;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class REVPneumatics extends Pneumatics {

    private final PneumaticHub pneumaticHub;

    public REVPneumatics(Config config,
                         PneumaticHub pneumaticHub) {
        super(config.name());
        this.pneumaticHub = pneumaticHub;
        SmartDashboard.putData("Compressor", pneumaticHub.makeCompressor());

    }

    @Override
    public boolean isCompressorOn() {
        return pneumaticHub.getCompressor();
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
}
