package digilib.claws;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import static digilib.claws.ClawState.*;

public class ClawTelemetry {
    private final StringPublisher clawStatePublisher;

    public ClawTelemetry(String name,
                         ClawValue clawValueWhenSolenoidOn) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getStringTopic("Solenoid On, Claw Value").publish().set(clawValueWhenSolenoidOn.name());
        clawStatePublisher = table.getStringTopic("State").publish();
    }

    public void telemeterize(ClawState state){
        clawStatePublisher.set(state.getClawValue().name());
    }
}
