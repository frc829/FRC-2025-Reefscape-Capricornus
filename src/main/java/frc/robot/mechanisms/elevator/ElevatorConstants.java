package frc.robot.mechanisms.elevator;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class ElevatorConstants {
    private final String name;
    private final Distance maxHeight;
    private final Distance minHeight;
    private final LinearVelocity maxVelocity;
    private final LinearAcceleration maxAcceleration;
    private final Voltage ks;
    private final Voltage kg;
    private final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka;
    private final Distance drumRadius;
    private final double reduction;
    private final Distance startingHeight;
    private final Distance positionStdDev;
    private final LinearVelocity velocityStdDev;

    public ElevatorConstants(
            String name,
            Distance maxHeight,
            Distance minHeight,
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration,
            Voltage ks,
            Voltage kg,
            Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka,
            Distance drumRadius,
            double reduction,
            Distance startingHeight,
            Distance positionStdDev,
            LinearVelocity velocityStdDev) {
        this.name = name;
        this.maxHeight = maxHeight;
        this.minHeight = minHeight;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
        this.drumRadius = drumRadius;
        this.reduction = reduction;
        this.startingHeight = startingHeight;
        this.positionStdDev = positionStdDev;
        this.velocityStdDev = velocityStdDev;
    }

    public Distance getMaxHeight() {
        return maxHeight;
    }

    public Distance getMinHeight() {
        return minHeight;
    }

    public Voltage getKs() {
        return ks;
    }

    public Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> getKv() {
        return kv;
    }

    public Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> getKa() {
        return ka;
    }

    public Voltage getKg() {
        return kg;
    }

    public Distance getDrumRadius() {
        return drumRadius;
    }

    public double getReduction() {
        return reduction;
    }

    public Distance getStartingHeight() {
        return startingHeight;
    }

    public Distance getPositionStdDev() {
        return positionStdDev;
    }

    public LinearVelocity getVelocityStdDev() {
        return velocityStdDev;
    }

    public String getName() {
        return name;
    }

    public LinearVelocity getMaxVelocity() {
        return maxVelocity;
    }

    public LinearAcceleration getMaxAcceleration() {
        return maxAcceleration;
    }
}
