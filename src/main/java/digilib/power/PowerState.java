package digilib.power;

import edu.wpi.first.units.measure.*;

import java.util.List;
import java.util.stream.IntStream;

import static edu.wpi.first.units.Units.*;

public class PowerState {
    private final MutVoltage voltage = Volts.mutable(0.0);
    private final MutTemperature temperature = Celsius.mutable(0.0);
    private final List<MutCurrent> currents;
    private final MutCurrent totalCurrent = Amps.mutable(0.0);
    private final MutPower totalPower = Watts.mutable(0.0);
    private final MutEnergy totalEnergy = Joules.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);


    public PowerState(int channels) {
        currents = IntStream.range(0, channels)
                .mapToObj(channel -> Amps.mutable(0.0))
                .toList();
    }

    public MutVoltage getVoltage() {
        return voltage;
    }

    public MutTemperature getTemperature() {
        return temperature;
    }

    public List<MutCurrent> getCurrents() {
        return currents;
    }

    public MutCurrent getTotalCurrent() {
        return totalCurrent;
    }

    public MutPower getTotalPower() {
        return totalPower;
    }

    public MutEnergy getTotalEnergy() {
        return totalEnergy;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public PowerState withVoltage(double voltageVolts) {
        voltage.mut_setMagnitude(voltageVolts);
        return this;
    }

    public PowerState withTemperature(double temperatureCelsius) {
        temperature.mut_setMagnitude(temperatureCelsius);
        return this;
    }

    public PowerState withCurrents(double... currentsInAmps) {
        for(int i = 0; i < currentsInAmps.length; i++) {
            currents.get(i).mut_setBaseUnitMagnitude(currentsInAmps[i]);
        }
        return this;
    }

    public PowerState withTotalCurrent(double totalCurrentAmps) {
        totalCurrent.mut_setMagnitude(totalCurrentAmps);
        return this;
    }

    public PowerState withTotalPower(double powerWatts) {
        totalPower.mut_setMagnitude(powerWatts);
        return this;
    }

    public PowerState withTotalEnergy(double energyJoules) {
        totalEnergy.mut_setMagnitude(energyJoules);
        return this;
    }

    public void withTimeStamp(double seconds){
        timestamp.mut_setMagnitude(seconds);
    }
}
