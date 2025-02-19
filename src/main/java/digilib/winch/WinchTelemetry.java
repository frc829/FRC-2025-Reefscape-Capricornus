package digilib.winch;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static digilib.DigiMath.roundToDecimal;
import static edu.wpi.first.units.Units.*;

public class WinchTelemetry {
    private final DoublePublisher dutyCycle;
    private final DoublePublisher voltage;
    private final DoublePublisher timestamp;

    public WinchTelemetry(String name) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        dutyCycle = table.getDoubleTopic("Duty Cycle").publish();
        voltage = table.getDoubleTopic("Voltage").publish();
        timestamp = table.getDoubleTopic("Timestamp").publish();
    }

    public void telemeterize(WinchState state) {
        dutyCycle.set(roundToDecimal(state.getDutyCycle().in(Value), 2));
        voltage.set(roundToDecimal(state.getVoltage().baseUnitMagnitude(), 2));
        timestamp.set(roundToDecimal(state.getTimestamp().baseUnitMagnitude(), 2));
    }
}
