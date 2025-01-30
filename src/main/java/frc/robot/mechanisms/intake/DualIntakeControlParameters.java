package frc.robot.mechanisms.intake;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public class DualIntakeControlParameters {

    private final Voltage intake0ks;
    private final Voltage intake1ks;
    private final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> intake0kv;
    private final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> intake1kv;
    private final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> intake0ka;
    private final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> intake1ka;
    private final Time updatePeriod;
    private final DualIntakeState currentState = new DualIntakeState();


    public DualIntakeControlParameters(
            Voltage intake0ks,
            Voltage intake1ks,
            Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> intake0kv,
            Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> intake1kv,
            Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> intake0ka,
            Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> intake1ka,
            Time updatePeriod) {
        this.intake0ks = intake0ks;
        this.intake1ks = intake1ks;
        this.intake0kv = intake0kv;
        this.intake1kv = intake1kv;
        this.intake0ka = intake0ka;
        this.intake1ka = intake1ka;
        this.updatePeriod = updatePeriod;
    }

    public DualIntakeState getCurrentState() {
        return currentState;
    }

    public Voltage getIntake0ks() {
        return intake0ks;
    }

    public Voltage getIntake1ks() {
        return intake1ks;
    }

    public Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> getIntake0kv() {
        return intake0kv;
    }

    public Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> getIntake1kv() {
        return intake1kv;
    }

    public Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> getIntake0ka() {
        return intake0ka;
    }

    public Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> getIntake1ka() {
        return intake1ka;
    }

    public Time getUpdatePeriod() {
        return updatePeriod;
    }

    public DualIntakeControlParameters withIntakeState(DualIntakeState state) {
        currentState.withIntakeState(state);
        return this;
    }

}
