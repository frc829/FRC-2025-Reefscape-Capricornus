package frc.robot.mechanisms.intake;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class IntakeMotorConstants {

    private final Voltage ks;
    private final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka;
    private final LinearVelocity maxAngularVelocity;
    private final LinearAcceleration maxAngularAcceleration;
    private final Distance wheelRadius;
    private final double reduction;
    private final LinearVelocity velocityStdDev;


    public IntakeMotorConstants(
            Voltage ks,
            Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka,
            Time updatePeriod,
            Distance wheelRadius,
            double reduction,
            LinearVelocity maxAngularVelocity,
            LinearAcceleration maxAngularAcceleration,
            LinearVelocity velocityStdDev) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.wheelRadius = wheelRadius;
        this.reduction = reduction;
        this.velocityStdDev = velocityStdDev;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
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

    public LinearVelocity getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public LinearAcceleration getMaxAngularAcceleration() {
        return maxAngularAcceleration;
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
}
