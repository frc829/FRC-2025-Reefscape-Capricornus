package digilib.units.currentperangvel;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public final class CurrentPerAngularVelocityUnit extends PerUnit<CurrentUnit, AngularVelocityUnit> {

    public static final CurrentPerAngularVelocityUnit AmpsPerRadiansPerSecond = new CurrentPerAngularVelocityUnit(Amps, RadiansPerSecond);
    public static final CurrentPerAngularVelocityUnit AmpsPerRotationsPerSecond = new CurrentPerAngularVelocityUnit(Amps, RotationsPerSecond);


    private static final CombinatoryUnitCache<CurrentUnit, AngularVelocityUnit, CurrentPerAngularVelocityUnit> cache =
            new CombinatoryUnitCache<>(CurrentPerAngularVelocityUnit::new);

    CurrentPerAngularVelocityUnit(CurrentUnit current, AngularVelocityUnit angularVelocity) {
        super(
                current.isBaseUnit() && angularVelocity.isBaseUnit()
                        ? null
                        : combine(current.getBaseUnit(), angularVelocity.getBaseUnit()),
                current,
                angularVelocity);
    }

    /**
     * Combines an energy and a time unit to form a unit of power.
     *
     * @param current         the unit of current
     * @param angularVelocity the unit of angular velocity
     * @return the combined unit of power
     */
    public static CurrentPerAngularVelocityUnit combine(CurrentUnit current, AngularVelocityUnit angularVelocity) {
        return cache.combine(current, angularVelocity);
    }

    @Override
    public CurrentPerAngularVelocityUnit getBaseUnit() {
        return (CurrentPerAngularVelocityUnit) super.getBaseUnit();
    }

    @Override
    public CurrentPerAngularVelocity of(double magnitude) {
        return new ImmutableCurrentPerAngularVelocity(magnitude, toBaseUnits(magnitude), this);
    }

    @Override
    public CurrentPerAngularVelocity ofBaseUnits(double baseUnitMagnitude) {
        return new ImmutableCurrentPerAngularVelocity(fromBaseUnits(baseUnitMagnitude), baseUnitMagnitude, this);
    }

    @Override
    public CurrentPerAngularVelocity zero() {
        return (CurrentPerAngularVelocity) super.zero();
    }

    @Override
    public CurrentPerAngularVelocity one() {
        return (CurrentPerAngularVelocity) super.one();
    }

    @Override
    public MutCurrentPerAngularVelocity mutable(double initialMagnitude) {
        return new MutCurrentPerAngularVelocity(initialMagnitude, toBaseUnits(initialMagnitude), this);
    }

    @Override
    public VelocityUnit<CurrentPerAngularVelocityUnit> per(TimeUnit time) {
        return VelocityUnit.combine(this, time);
    }

    /**
     * Creates a ratio unit between this unit and an arbitrary other unit.
     *
     * @param other the other unit
     * @param <U>   the type of the other unit
     * @return the ratio unit
     */
    public <U extends Unit> PerUnit<CurrentPerAngularVelocityUnit, U> per(U other) {
        return PerUnit.combine(this, other);
    }

    /**
     * Converts a measurement value in terms of another power unit to this unit.
     *
     * @param magnitude the magnitude of the measurement in terms of the other power unit
     * @param otherUnit the other power unit
     * @return the value of the measurement in terms of this unit
     */
    public double convertFrom(double magnitude, CurrentPerAngularVelocityUnit otherUnit) {
        return fromBaseUnits(otherUnit.toBaseUnits(magnitude));
    }
}