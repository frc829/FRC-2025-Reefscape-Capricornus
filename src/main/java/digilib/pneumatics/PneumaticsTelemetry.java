package digilib.pneumatics;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PneumaticsTelemetry {
    private final DoublePublisher timestamp;
    private final BooleanPublisher compressorOnPublisher;

    public PneumaticsTelemetry(
            String name,
            PneumaticsBase pneumaticsBase) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        this.timestamp = table.getDoubleTopic("Timestamp").publish();
        this.compressorOnPublisher = table.getBooleanTopic("Compressor On").publish();
        SmartDashboard.putData("Compressor", pneumaticsBase.makeCompressor());
    }

    public void telemeterize(PneumaticsState state) {
        timestamp.set(state.getTimestamp().baseUnitMagnitude());
        compressorOnPublisher.set(state.isCompressorOn());
    }
}
