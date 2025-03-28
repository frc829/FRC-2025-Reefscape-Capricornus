package digilib.power;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.PowerDistribution.*;

public class Power {

    public record Config(String name, int module, ModuleType moduleType){}

    private final PowerDistribution powerDistribution;

    private final DoublePublisher voltsPublisher;
    private final DoublePublisher temperaturePublisher;
    private final DoubleArrayPublisher currentsPublisher;
    private final DoublePublisher totalCurrentPublisher;
    private final DoublePublisher totalPowerPublisher;
    private final DoublePublisher totalEnergyPublisher;

    public Power(Config config) {
        powerDistribution = new PowerDistribution(config.module(), config.moduleType());
        NetworkTable table = NetworkTableInstance.getDefault().getTable(config.name());
        this.voltsPublisher = table.getDoubleTopic("Voltage [volts]").publish();
        this.temperaturePublisher = table.getDoubleTopic("Temperature [Celsius]").publish();
        this.currentsPublisher = table.getDoubleArrayTopic("Currents [amps]").publish();
        this.totalCurrentPublisher = table.getDoubleTopic("TotalCurrent [amps]").publish();
        this.totalPowerPublisher = table.getDoubleTopic("TotalPower [watts]").publish();
        this.totalEnergyPublisher = table.getDoubleTopic("TotalEnergy [joules]").publish();
        SmartDashboard.putData("Main Power", powerDistribution);
    }

    public double getVolts() {
        return powerDistribution.getVoltage();
    }

    public double getTemperature() {
        return powerDistribution.getTemperature();
    }

    public double[] getCurrentsAmps() {
        return powerDistribution.getAllCurrents();
    }

    public double getTotalCurrentAmps() {
        return powerDistribution.getTotalCurrent();
    }

    public double getTotalEnergy() {
        return powerDistribution.getTotalEnergy();
    }

    public double getTotalPower() {
        return powerDistribution.getTotalPower();
    }

    public void clearStickyFaults(){
        powerDistribution.clearStickyFaults();
    }

    public void update(){
        voltsPublisher.set(getVolts());
        temperaturePublisher.set(getTemperature());
        currentsPublisher.set(getCurrentsAmps());
        totalCurrentPublisher.set(getTotalCurrentAmps());
        totalPowerPublisher.set(getTotalPower());
        totalEnergyPublisher.set(getTotalEnergy());
    }
}
