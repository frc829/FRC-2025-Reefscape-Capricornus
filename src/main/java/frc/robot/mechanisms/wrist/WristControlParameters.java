package frc.robot.mechanisms.wrist;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class WristControlParameters {

    private final Angle maxAngle;
    private final Angle minAngle;
    private final AngularVelocity maxVelocity;
    private final AngularAcceleration maxAcceleration;
    private final Voltage ks;
    private final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka;
    private final WristState currentState = new WristState();


    public WristControlParameters(
            Angle maxAngle,
            Angle minAngle,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration,
            Voltage ks,
            Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka) {
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.ks = ks;
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


}
