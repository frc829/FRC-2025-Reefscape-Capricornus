package digilib.linearsystem;

import edu.wpi.first.math.util.Units;

public class Motor {

    public final double nominalVoltageVolts;
    public final double freeCurrentAmps;
    public final double stallCurrentAmps;
    public final double freeSpeedRadPerSec;
    public final double stallTorqueNewtonMeters;
    public final double ROhms;
    public final double KeVoltsPerRadPerSec;
    public final double KtNMPerAmp;

    public Motor(
            double nominalVoltageVolts,
            double stallTorqueNewtonMeters,
            double stallCurrentAmps,
            double freeCurrentAmps,
            double freeSpeedRadPerSec) {
        this.nominalVoltageVolts = nominalVoltageVolts;
        this.freeCurrentAmps = freeCurrentAmps;
        this.stallCurrentAmps = stallCurrentAmps;
        this.freeSpeedRadPerSec = freeSpeedRadPerSec;
        this.stallTorqueNewtonMeters = stallTorqueNewtonMeters;
        ROhms = nominalVoltageVolts / stallCurrentAmps;
        KeVoltsPerRadPerSec = (nominalVoltageVolts - freeCurrentAmps * ROhms) / freeSpeedRadPerSec;
        KtNMPerAmp = stallTorqueNewtonMeters / stallCurrentAmps;
    }

    public double getCurrentAmps(double voltageVolts, double omegaRadPerSec) {
        return (voltageVolts - KeVoltsPerRadPerSec * omegaRadPerSec) / ROhms;
    }

    public double getOmegaRadPerSec(double voltageVolts, double currentAmps) {
        return (voltageVolts - currentAmps * ROhms) / KeVoltsPerRadPerSec;
    }

    public double getVoltageVolts(double omegaRadPerSec, double currentAmps) {
        return currentAmps * ROhms + KeVoltsPerRadPerSec * omegaRadPerSec;
    }

    public double getPowerWattsFromVoltage(double voltageVolts, double omegaRadPerSec){
        return voltageVolts * getCurrentAmps(voltageVolts, omegaRadPerSec);
    }

    public double getPowerWattsFromCurrent(double currentAmps, double omegaRadPerSec){
        return getVoltageVolts(currentAmps, omegaRadPerSec) * currentAmps;
    }

    public static final Motor NEO550 = new Motor(12, 0.97, 100, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(11000.0));

    public static final Motor NEO = new Motor(12, 2.6, 105, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(5676));

    public static final Motor NeoVortex = new Motor(12, 3.60, 211, 3.6, Units.rotationsPerMinuteToRadiansPerSecond(6784.0));

    public static final Motor KrakenX60 = new Motor(12, 7.09, 366, 2, Units.rotationsPerMinuteToRadiansPerSecond(6000));

    public static final Motor KrakenX60Foc = new Motor(12, 9.37, 483, 2, Units.rotationsPerMinuteToRadiansPerSecond(5800));

}
