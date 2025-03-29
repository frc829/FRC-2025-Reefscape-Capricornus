package digilib.claws;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public abstract class Claw {


    public enum Value {
        OPEN,
        CLOSED

    }

    public record Config(String name, PneumaticsModuleType moduleType, Value solenoidOnValue) {
    }

    protected final Value valueWhenSolenoidOn;
    private final StringPublisher clawValuePublisher;
    protected Value value = Value.CLOSED;

    @SuppressWarnings("resource")
    public Claw(String name,
                Value valueWhenSolenoidOn) {
        this.valueWhenSolenoidOn = valueWhenSolenoidOn;
        NetworkTable table = NetworkTableInstance
                .getDefault()
                .getTable(name);
        table.getStringTopic("Solenoid On, Claw Value")
                .publish()
                .set(valueWhenSolenoidOn.name());
        clawValuePublisher = table
                .getStringTopic("State")
                .publish();
    }

    public Value getValue(){
        return value;
    }

    public abstract void setValue(Value state);

    public abstract void toggle();

    public void update() {
        clawValuePublisher.set(value.name());
    }
}
