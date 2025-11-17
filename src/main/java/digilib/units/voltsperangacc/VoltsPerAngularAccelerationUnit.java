package digilib.units.voltsperangacc;

import digilib.units.voltsperangvel.VoltsPerAngularVelocityUnit;
import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public final class VoltsPerAngularAccelerationUnit extends PerUnit<VoltageUnit, AngularAccelerationUnit> {

    public static final VoltsPerAngularAccelerationUnit VoltsPerRadiansPerSecondPerSecond = new VoltsPerAngularAccelerationUnit(Volts, RadiansPerSecondPerSecond);
    public static final VoltsPerAngularAccelerationUnit VoltsPerRotationsPerSecondPerSecond = new VoltsPerAngularAccelerationUnit(Volts, RotationsPerSecondPerSecond);


    private static final CombinatoryUnitCache<VoltageUnit, AngularAccelerationUnit, VoltsPerAngularAccelerationUnit> cache =
            new CombinatoryUnitCache<>(VoltsPerAngularAccelerationUnit::new);

    VoltsPerAngularAccelerationUnit(VoltageUnit voltage, AngularAccelerationUnit angularAcceleration) {
        super(
                voltage.isBaseUnit() && angularAcceleration.isBaseUnit()
                        ? null
                        : combine(voltage.getBaseUnit(), angularAcceleration.getBaseUnit()),
                voltage,
                angularAcceleration);
    }

    /**
     * Combines an energy and a time unit to form a unit of power.
     *
     * @param voltage         the unit of voltage
     * @param angularAcceleration the unit of angular acceleration
     * @return the combined unit of power
     */
    public static VoltsPerAngularAccelerationUnit combine(VoltageUnit voltage, AngularAccelerationUnit angularAcceleration) {
        return cache.combine(voltage, angularAcceleration);
    }

    @Override
    public VoltsPerAngularAccelerationUnit getBaseUnit() {
        return (VoltsPerAngularAccelerationUnit) super.getBaseUnit();
    }

    @Override
    public VoltsPerAngularAcceleration of(double magnitude) {
        return new ImmutableVoltsPerAngularAcceleration(magnitude, toBaseUnits(magnitude), this);
    }

    @Override
    public VoltsPerAngularAcceleration ofBaseUnits(double baseUnitMagnitude) {
        return new ImmutableVoltsPerAngularAcceleration(fromBaseUnits(baseUnitMagnitude), baseUnitMagnitude, this);
    }

    @Override
    public VoltsPerAngularAcceleration zero() {
        return (VoltsPerAngularAcceleration) super.zero();
    }

    @Override
    public VoltsPerAngularAcceleration one() {
        return (VoltsPerAngularAcceleration) super.one();
    }

    @Override
    public MutVoltsPerAngularAcceleration mutable(double initialMagnitude) {
        return new MutVoltsPerAngularAcceleration(initialMagnitude, toBaseUnits(initialMagnitude), this);
    }

    @Override
    public VelocityUnit<VoltsPerAngularAccelerationUnit> per(TimeUnit time) {
        return VelocityUnit.combine(this, time);
    }

    /**
     * Creates a ratio unit between this unit and an arbitrary other unit.
     *
     * @param other the other unit
     * @param <U>   the type of the other unit
     * @return the ratio unit
     */
    public <U extends Unit> PerUnit<VoltsPerAngularAccelerationUnit, U> per(U other) {
        return PerUnit.combine(this, other);
    }

    /**
     * Converts a measurement value in terms of another power unit to this unit.
     *
     * @param magnitude the magnitude of the measurement in terms of the other power unit
     * @param otherUnit the other power unit
     * @return the value of the measurement in terms of this unit
     */
    public double convertFrom(double magnitude, VoltsPerAngularAccelerationUnit otherUnit) {
        return fromBaseUnits(otherUnit.toBaseUnits(magnitude));
    }
}