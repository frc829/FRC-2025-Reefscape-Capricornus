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

    public static final Motor CIM = new Motor(12, 2.42, 133, 2.7, Units.rotationsPerMinuteToRadiansPerSecond(5310));

    public static final Motor Vex775Pro = new Motor(12, 0.71, 134, 0.7, Units.rotationsPerMinuteToRadiansPerSecond(18730));

    public static final Motor NEO = new Motor(12, 2.6, 105, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(5676));

    public static final Motor MiniCIM = new Motor(12, 1.41, 89, 3, Units.rotationsPerMinuteToRadiansPerSecond(5840));

    public static final Motor Bag = new Motor(12, 0.43, 53, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(13180));

    public static final Motor AndymarkRs775_125 = new Motor(12, 0.28, 18, 1.6, Units.rotationsPerMinuteToRadiansPerSecond(5800.0));

    public static final Motor BanebotsRs775 = new Motor(12, 0.72, 97, 2.7, Units.rotationsPerMinuteToRadiansPerSecond(13050.0));

    public static final Motor Andymark9015 = new Motor(12, 0.36, 71, 3.7, Units.rotationsPerMinuteToRadiansPerSecond(14270.0));

    public static final Motor BanebotsRs550 = new Motor(12, 0.38, 84, 0.4, Units.rotationsPerMinuteToRadiansPerSecond(19000.0));

    public static final Motor NEO550 = new Motor(12, 0.97, 100, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(11000.0));

    public static final Motor Falcon500 = new Motor(12, 4.69, 257, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6380));

    public static final Motor Falcon500Foc = new Motor(12, 5.84, 304, 1.5, Units.rotationsPerMinuteToRadiansPerSecond(6080.0));

    public static final Motor RomiBuiltIn = new Motor(4.5, 0.1765, 1.25, 0.13, Units.rotationsPerMinuteToRadiansPerSecond(150.0));

    public static final Motor KrakenX60 = new Motor(12, 7.09, 366, 2, Units.rotationsPerMinuteToRadiansPerSecond(6000));

    public static final Motor KrakenX60Foc = new Motor(12, 9.37, 483, 2, Units.rotationsPerMinuteToRadiansPerSecond(5800));

    public static final Motor NeoVortex = new Motor(12, 3.60, 211, 3.6, Units.rotationsPerMinuteToRadiansPerSecond(6784.0));
}
