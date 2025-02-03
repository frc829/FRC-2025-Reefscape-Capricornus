package frc.robot.mechanisms.claws;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import static edu.wpi.first.units.Units.Seconds;

public class ClawTelemetry {
    private final NetworkTable clawStateTable;
    private final DoublePublisher timestampPublisher;
    private final StringPublisher clawStatePublisher;

    public ClawTelemetry(String name) {
        clawStateTable = NetworkTableInstance.getDefault().getTable(name);
        timestampPublisher = clawStateTable.getDoubleTopic("Timestamp").publish();
        clawStatePublisher = clawStateTable.getStringTopic("State").publish();
    }

    public void telemeterize(ClawState state){
        timestampPublisher.set(state.getTimestamp().in(Seconds));
        clawStatePublisher.set(state.getClawValue().name());
    }

}
