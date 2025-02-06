package digilib.power;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
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
    private PowerDistributionFaults faults = new PowerDistributionFaults(0);
    private PowerDistributionStickyFaults stickyFaults = new PowerDistributionStickyFaults(0);

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

    public PowerDistributionFaults getFaults() {
        return faults;
    }

    public PowerDistributionStickyFaults getStickyFaults() {
        return stickyFaults;
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
        IntStream.range(0, currents.size())
                .forEachOrdered(i -> this.currents.get(i).mut_setMagnitude(currentsInAmps[i]));
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

    public PowerState withFaults(PowerDistributionFaults faults) {
        this.faults = faults;
        return this;
    }

    public PowerState withStickyFaults(PowerDistributionStickyFaults stickyFaults) {
        this.stickyFaults = stickyFaults;
        return this;
    }

    public PowerState withState(PowerState state) {
        voltage.mut_replace(state.getVoltage());
        temperature.mut_replace(state.getTemperature());
        IntStream.range(0, currents.size())
                .forEachOrdered(i -> currents.get(i).mut_replace(state.getCurrents().get(i)));
        totalCurrent.mut_replace(state.getTotalCurrent());
        totalPower.mut_replace(state.getTotalPower());
        totalEnergy.mut_replace(state.getTotalEnergy());
        faults = state.getFaults();
        stickyFaults = state.getStickyFaults();
        return this;
    }


    @Override
    public PowerState clone() {
        try {
            PowerState toReturn = (PowerState) super.clone();
            toReturn.voltage.mut_replace(voltage);
            toReturn.temperature.mut_replace(temperature);
            toReturn.currents.replaceAll(i -> i.mutableCopy());
            toReturn.totalCurrent.mut_replace(totalCurrent);
            toReturn.totalPower.mut_replace(totalPower);
            toReturn.totalEnergy.mut_replace(totalEnergy);
            toReturn.faults = faults;
            toReturn.stickyFaults = stickyFaults;

            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }

}
