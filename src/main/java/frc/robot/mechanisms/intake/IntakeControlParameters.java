package frc.robot.mechanisms.intake;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class IntakeControlParameters {

    private final LinearVelocity maxVelocity;
    private final LinearAcceleration maxAcceleration;
    private final Voltage ks;
    private final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka;
    private final IntakeState currentState = new IntakeState();


    public IntakeControlParameters(
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration,
            Voltage ks,
            Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }

    public LinearVelocity getMaxVelocity() {
        return maxVelocity;
    }

    public LinearAcceleration getMaxAcceleration() {
        return maxAcceleration;
    }

    public IntakeState getCurrentState() {
        return currentState;
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

    public IntakeControlParameters withIntakeState(IntakeState state) {
        currentState.withIntakeState(state);
        return this;
    }


}
