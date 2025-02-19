package digilib.winch;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class WinchState {

    private final MutDimensionless dutyCycle = Percent.mutable(0.0);
    private final MutTime timestamp = Seconds.mutable(0.0);
    private final MutVoltage voltage = Volts.mutable(0.0);


    public Dimensionless getDutyCycle() {
        return dutyCycle;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public Voltage getVoltage() {
        return voltage;
    }

    public WinchState withDutyCycle(double dutyCycle){
        this.dutyCycle.mut_setBaseUnitMagnitude(dutyCycle);
        return this;
    }

    public WinchState withTimestamp(double seconds){
        this.timestamp.mut_setBaseUnitMagnitude(seconds);
        return this;
    }

    public WinchState withVoltage(double voltage){
        this.voltage.mut_setBaseUnitMagnitude(voltage);
        return this;
    }
}
