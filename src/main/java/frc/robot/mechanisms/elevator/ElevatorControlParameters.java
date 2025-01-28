package frc.robot.mechanisms.elevator;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class ElevatorControlParameters {

    private final Distance maxHeight;
    private final Distance minHeight;
    private final Voltage ks;
    private final Voltage kg;
    private final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv;
    private final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka;
    private final Time updatePeriod;
    private final ElevatorState currentState = new ElevatorState();


    public ElevatorControlParameters(
            Distance maxHeight,
            Distance minHeight,
            Voltage ks,
            Voltage kg,
            Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka,
            Time updatePeriod
    ) {
        this.maxHeight = maxHeight;
        this.minHeight = minHeight;
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
        this.updatePeriod = updatePeriod;
    }

    public Distance getMaxHeight() {
        return maxHeight;
    }

    public Distance getMinHeight() {
        return minHeight;
    }

    public ElevatorState getCurrentState() {
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

    public Voltage getKg() {
        return kg;
    }

    public ElevatorControlParameters withElevatorState(ElevatorState state) {
        currentState.withElevatorState(state);
        return this;
    }


    public Time getUpdatePeriod() {
        return updatePeriod;
    }
}
