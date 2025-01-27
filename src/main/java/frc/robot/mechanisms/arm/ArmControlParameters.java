package frc.robot.mechanisms.arm;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class ArmControlParameters {

    private final Angle maxAngle;
    private final Angle minAngle;
    private final AngularVelocity maxVelocity;
    private final AngularAcceleration maxAcceleration;
    private final Voltage ks;
    private final Voltage kg;
    private final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka;
    private final ArmState currentState = new ArmState();


    public ArmControlParameters(
            Angle maxAngle,
            Angle minAngle,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration,
            Voltage ks,
            Voltage kg,
            Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka) {
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
    }

    public Angle getMaxAngle() {
        return maxAngle;
    }

    public Angle getMinAngle() {
        return minAngle;
    }

    public AngularVelocity getMaxVelocity() {
        return maxVelocity;
    }

    public AngularAcceleration getMaxAcceleration() {
        return maxAcceleration;
    }

    public ArmState getCurrentState() {
        return currentState;
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

    public ArmControlParameters withArmState(ArmState state) {
        currentState.withArmState(state);
        return this;
    }


}
