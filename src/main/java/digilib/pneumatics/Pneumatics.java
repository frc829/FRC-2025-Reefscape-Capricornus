package digilib.pneumatics;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class Pneumatics {

    public record Config(String name) {
    }

    private final BooleanPublisher compressorOnPublisher;

    Pneumatics(String name) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        this.compressorOnPublisher = table.getBooleanTopic("Compressor On").publish();
    }

    public abstract boolean isCompressorOn();

    public abstract void clearStickyFaults();

    public abstract void turnOnCompressor();

    public abstract void turnOffCompressor();

    public void update(){
        compressorOnPublisher.set(isCompressorOn());
    }
}
