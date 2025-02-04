package frc.robot.mechanisms.intakeWheel;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class IntakeWheelConstants {

    private final Voltage ks;
    private final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka;
    private final Distance wheelRadius;
    private final double reduction;
    private final LinearVelocity velocityStdDev;
    private final Time updatePeriod;
    private final LinearAcceleration maxAcceleration;


    public IntakeWheelConstants(
            Voltage ks,
            Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka,
            Distance wheelRadius,
            double reduction,
            LinearVelocity velocityStdDev,
            Time updatePeriod,
            LinearAcceleration maxAcceleration) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.wheelRadius = wheelRadius;
        this.reduction = reduction;
        this.velocityStdDev = velocityStdDev;
        this.updatePeriod = updatePeriod;
        this.maxAcceleration = maxAcceleration;
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

    public LinearAcceleration getMaxAcceleration() {
        return maxAcceleration;
    }
}
