package frc.robot.mechanisms.arm;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class ArmControlParameters {

    private final Angle maxAngle;
    private final Angle minAngle;
    private final Voltage ks;
    private final Voltage kg;
    private final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka;
    private final Time updatePeriod;
    private final ArmState currentState = new ArmState();


    public ArmControlParameters(
            Angle maxAngle,
            Angle minAngle,
            Voltage ks,
            Voltage kg,
            Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka,
            Time updatePeriod) {
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
        this.updatePeriod = updatePeriod;
    }

    public Angle getMaxAngle() {
        return maxAngle;
    }

    public Angle getMinAngle() {
        return minAngle;
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


    public Time getUpdatePeriod() {
        return updatePeriod;
    }
}
