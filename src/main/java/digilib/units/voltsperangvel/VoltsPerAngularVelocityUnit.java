package digilib.units.voltsperangvel;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public final class VoltsPerAngularVelocityUnit extends PerUnit<VoltageUnit, AngularVelocityUnit> {

    public static final VoltsPerAngularVelocityUnit VoltsPerRadiansPerSecond = new VoltsPerAngularVelocityUnit(Volts, RadiansPerSecond);
    public static final VoltsPerAngularVelocityUnit VoltsPerRotationsPerSecond = new VoltsPerAngularVelocityUnit(Volts, RotationsPerSecond);

    private static final CombinatoryUnitCache<VoltageUnit, AngularVelocityUnit, VoltsPerAngularVelocityUnit> cache =
            new CombinatoryUnitCache<>(VoltsPerAngularVelocityUnit::new);

    VoltsPerAngularVelocityUnit(VoltageUnit voltage, AngularVelocityUnit angularVelocity) {
        super(
                voltage.isBaseUnit() && angularVelocity.isBaseUnit()
                        ? null
                        : combine(voltage.getBaseUnit(), angularVelocity.getBaseUnit()),
                voltage,
                angularVelocity);
    }

    /**
     * Combines an energy and a time unit to form a unit of power.
     *
     * @param voltage         the unit of voltage
     * @param angularVelocity the unit of angular velocity
     * @return the combined unit of power
     */
    public static VoltsPerAngularVelocityUnit combine(VoltageUnit voltage, AngularVelocityUnit angularVelocity) {
        return cache.combine(voltage, angularVelocity);
    }

    @Override
    public VoltsPerAngularVelocityUnit getBaseUnit() {
        return (VoltsPerAngularVelocityUnit) super.getBaseUnit();
    }

    @Override
    public VoltsPerAngularVelocity of(double magnitude) {
        return new ImmutableVoltsPerAngularVelocity(magnitude, toBaseUnits(magnitude), this);
    }

    @Override
    public VoltsPerAngularVelocity ofBaseUnits(double baseUnitMagnitude) {
        return new ImmutableVoltsPerAngularVelocity(fromBaseUnits(baseUnitMagnitude), baseUnitMagnitude, this);
    }

    @Override
    public VoltsPerAngularVelocity zero() {
        return (VoltsPerAngularVelocity) super.zero();
    }

    @Override
    public VoltsPerAngularVelocity one() {
        return (VoltsPerAngularVelocity) super.one();
    }

    @Override
    public MutVoltsPerAngularVelocity mutable(double initialMagnitude) {
        return new MutVoltsPerAngularVelocity(initialMagnitude, toBaseUnits(initialMagnitude), this);
    }

    @Override
    public VelocityUnit<VoltsPerAngularVelocityUnit> per(TimeUnit time) {
        return VelocityUnit.combine(this, time);
    }

    /**
     * Creates a ratio unit between this unit and an arbitrary other unit.
     *
     * @param other the other unit
     * @param <U>   the type of the other unit
     * @return the ratio unit
     */
    public <U extends Unit> PerUnit<VoltsPerAngularVelocityUnit, U> per(U other) {
        return PerUnit.combine(this, other);
    }

    /**
     * Converts a measurement value in terms of another power unit to this unit.
     *
     * @param magnitude the magnitude of the measurement in terms of the other power unit
     * @param otherUnit the other power unit
     * @return the value of the measurement in terms of this unit
     */
    public double convertFrom(double magnitude, VoltsPerAngularVelocityUnit otherUnit) {
        return fromBaseUnits(otherUnit.toBaseUnits(magnitude));
    }
}