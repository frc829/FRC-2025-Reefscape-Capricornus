package digilib.claws;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import static edu.wpi.first.units.Units.Seconds;

public class ClawTelemetry {
    private final DoublePublisher timestampPublisher;
    private final StringPublisher clawStatePublisher;

    public ClawTelemetry(String name,
                         ClawValue clawValueWhenSolenoidOn) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getStringTopic("Solenoid On, Claw Value").publish().set(clawValueWhenSolenoidOn.name());
        timestampPublisher = table.getDoubleTopic("Timestamp").publish();
        clawStatePublisher = table.getStringTopic("State").publish();
    }

    public void telemeterize(ClawState state){
        timestampPublisher.set(state.getTimestamp().in(Seconds));
        clawStatePublisher.set(state.getClawValue().name());
    }
}
