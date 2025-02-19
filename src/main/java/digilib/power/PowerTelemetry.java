package digilib.power;

import edu.wpi.first.networktables.*;
import edu.wpi.first.units.mutable.MutableMeasureBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PowerTelemetry {
    private final DoublePublisher timestamp;
    private final DoublePublisher voltage;
    private final DoublePublisher temperature;
    private final DoubleArrayPublisher currents;
    private final DoublePublisher totalCurrent;
    private final DoublePublisher totalPower;
    private final DoublePublisher totalEnergy;

    public PowerTelemetry(
            String name,
            PowerDistribution powerDistribution) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        this.timestamp = table.getDoubleTopic("Timestamp").publish();
        this.voltage = table.getDoubleTopic("Voltage").publish();
        this.temperature = table.getDoubleTopic("Temperature").publish();
        this.currents = table.getDoubleArrayTopic("Currents").publish();
        this.totalCurrent = table.getDoubleTopic("TotalCurrent").publish();
        this.totalPower = table.getDoubleTopic("TotalPower").publish();
        this.totalEnergy = table.getDoubleTopic("TotalEnergy").publish();
        SmartDashboard.putData("Main Power", powerDistribution);

    }

    public void telemeterize(PowerState state) {
        timestamp.set(state.getTimestamp().baseUnitMagnitude());
        voltage.set(state.getVoltage().baseUnitMagnitude());
        temperature.set(state.getTemperature().baseUnitMagnitude());
        currents.set(state.getCurrents().stream().mapToDouble(MutableMeasureBase::baseUnitMagnitude).toArray());
        totalCurrent.set(state.getTotalCurrent().baseUnitMagnitude());
        totalPower.set(state.getTotalPower().baseUnitMagnitude());
        totalEnergy.set(state.getTotalEnergy().baseUnitMagnitude());
    }
}
