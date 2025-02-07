package frc.robot.mechanisms.arm;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class
ArmConstants {

    private final Angle maxAngle;
    private final Angle minAngle;
    private final AngularVelocity maxAngularVelocity;
    private final AngularAcceleration maxAngularAcceleration;
    private final Voltage ks;
    private final Voltage kg;
    private final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka;
    private final Distance armLength;
    private final double reduction;
    private final Angle startingAngle;
    private final Angle positionStdDev;
    private final AngularVelocity velocityStdDev;

    public ArmConstants(
            Angle maxAngle,
            Angle minAngle,
            AngularVelocity maxAngularVelocity,
            AngularAcceleration maxAngularAcceleration,
            Voltage ks,
            Voltage kg,
            Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka,
            Distance armLength,
            double reduction,
            Angle startingAngle,
            Angle positionStdDev,
            AngularVelocity velocityStdDev) {
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
        this.armLength = armLength;
        this.reduction = reduction;
        this.startingAngle = startingAngle;
        this.positionStdDev = positionStdDev;
        this.velocityStdDev = velocityStdDev;
    }

    public Angle getMaxAngle() {
        return maxAngle;
    }

    public Angle getMinAngle() {
        return minAngle;
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

    public Voltage getKg() {
        return kg;
    }

    public Distance getArmLength() {
        return armLength;
    }

    public double getReduction() {
        return reduction;
    }

    public Angle getStartingAngle() {
        return startingAngle;
    }

    public Angle getPositionStdDev() {
        return positionStdDev;
    }

    public AngularVelocity getVelocityStdDev() {
        return velocityStdDev;
    }

    public AngularVelocity getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public AngularAcceleration getMaxAngularAcceleration() {
        return maxAngularAcceleration;
    }
}
