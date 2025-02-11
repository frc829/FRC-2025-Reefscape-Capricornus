package digilib.intakeWheel;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class IntakeWheelConstants {
    private final String name;
    private final Voltage ks;
    private final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka;
    private final Distance wheelRadius;
    private final double reduction;
    private final LinearVelocity velocityStdDev;
    private final Time updatePeriod;
    private final AngularVelocity maxVelocity;
    private final AngularAcceleration maxAcceleration;


    public IntakeWheelConstants(
            String name,
            Voltage ks,
            Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka,
            Distance wheelRadius,
            double reduction,
            LinearVelocity velocityStdDev,
            Time updatePeriod,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        this.name = name;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.wheelRadius = wheelRadius;
        this.reduction = reduction;
        this.velocityStdDev = velocityStdDev;
        this.updatePeriod = updatePeriod;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }


    public Voltage getKs() {
        return ks;
    }

    public Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> getKv() {
        return kv;
    }

    public Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> getKa() {
        return ka;
    }

    public Distance getWheelRadius() {
        return wheelRadius;
    }

    public double getReduction() {
        return reduction;
    }

    public LinearVelocity getVelocityStdDev() {
        return velocityStdDev;
    }

    public Time getUpdatePeriod() {
        return updatePeriod;
    }

    public AngularAcceleration getMaxAcceleration() {
        return maxAcceleration;
    }

    public String getName() {
        return name;
    }

    public AngularVelocity getMaxVelocity() {
        return maxVelocity;
    }
}
