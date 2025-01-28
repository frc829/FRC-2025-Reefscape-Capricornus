package frc.robot.mechanisms.wrist;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class WristControlParameters {

    private final Angle maxAngle;
    private final Angle minAngle;
    private final Voltage ks;
    private final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka;
    private final WristState currentState = new WristState();
    private final Time updatePeriod;


    public WristControlParameters(
            Angle maxAngle,
            Angle minAngle,
            Voltage ks,
            Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka,
            Time updatePeriod) {
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.ks = ks;
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

    public WristState getCurrentState() {
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

    public WristControlParameters withWristState(WristState state) {
        currentState.withWristState(state);
        return this;
    }


    public Time getUpdatePeriod() {
        return updatePeriod;
    }
}
