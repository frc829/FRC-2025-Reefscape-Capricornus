package digilib.cameraPower;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraPowerTelemetry {
    private final DoublePublisher timestamp;
    private final DoubleArrayPublisher voltages;
    private final DoubleArrayPublisher currents;

    public CameraPowerTelemetry(String name) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        this.timestamp = table.getDoubleTopic("Timestamp").publish();
        this.voltages = table.getDoubleArrayTopic("Voltages").publish();
        this.currents = table.getDoubleArrayTopic("Currents").publish();

    }

    public void telemeterize(CameraPowerState state) {
//        voltages.set(state.getVoltages().baseUnitMagnitude());
//        currents.set(state.getCurrents().stream().mapToDouble(MutableMeasureBase::baseUnitMagnitude).toArray());
    }
}
