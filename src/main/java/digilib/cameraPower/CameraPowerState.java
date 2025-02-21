package digilib.cameraPower;

import edu.wpi.first.units.measure.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static edu.wpi.first.units.Units.*;

public class CameraPowerState {
    private final Map<String, MutVoltage> voltages = new HashMap<>();
    private final Map<String, MutCurrent> currents = new HashMap<>();
    private final MutTime timestamp = Seconds.mutable(0.0);


    public CameraPowerState(List<String> channelNames) {
        channelNames.forEach(name -> voltages.put(name, Volts.mutable(0.0)));
        channelNames.forEach(name -> currents.put(name, Amps.mutable(0.0)));
    }

    public Map<String, MutVoltage> getVoltages() {
        return voltages;
    }

    public Map<String, MutCurrent> getCurrents() {
        return currents;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public void setVoltages(Map<String, Double> voltages) {
        voltages.keySet().forEach(key -> this.voltages.get(key).mut_setBaseUnitMagnitude(voltages.get(key)));
    }

    public void setCurrents(Map<String, Double> currents) {
        currents.keySet().forEach(key -> this.currents.get(key).mut_setBaseUnitMagnitude(currents.get(key)));

    }

    public void setTimeStamp(double seconds) {
        timestamp.mut_setMagnitude(seconds);
    }
}
