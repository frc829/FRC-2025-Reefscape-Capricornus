package digilib.units.currentperangacc;

import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public final class CurrentPerAngularAccelerationUnit extends PerUnit<CurrentUnit, AngularAccelerationUnit> {

    public static final CurrentPerAngularAccelerationUnit AmpsPerRadiansPerSecondPerSecond = new CurrentPerAngularAccelerationUnit(Amps, RadiansPerSecondPerSecond);
    public static final CurrentPerAngularAccelerationUnit AmpsPerRotationsPerSecondPerSecond = new CurrentPerAngularAccelerationUnit(Amps, RotationsPerSecondPerSecond);


    private static final CombinatoryUnitCache<CurrentUnit, AngularAccelerationUnit, CurrentPerAngularAccelerationUnit> cache =
            new CombinatoryUnitCache<>(CurrentPerAngularAccelerationUnit::new);

    CurrentPerAngularAccelerationUnit(CurrentUnit current, AngularAccelerationUnit angularAcceleration) {
        super(
                current.isBaseUnit() && angularAcceleration.isBaseUnit()
                        ? null
                        : combine(current.getBaseUnit(), angularAcceleration.getBaseUnit()),
                current,
                angularAcceleration);
    }

    /**
     * Combines an energy and a time unit to form a unit of power.
     *
     * @param current         the unit of current
     * @param angularAcceleration the unit of angular acceleration
     * @return the combined unit of power
     */
    public static CurrentPerAngularAccelerationUnit combine(CurrentUnit current, AngularAccelerationUnit angularAcceleration) {
        return cache.combine(current, angularAcceleration);
    }

    @Override
    public CurrentPerAngularAccelerationUnit getBaseUnit() {
        return (CurrentPerAngularAccelerationUnit) super.getBaseUnit();
    }

    @Override
    public CurrentPerAngularAcceleration of(double magnitude) {
        return new ImmutableCurrentPerAngularAcceleration(magnitude, toBaseUnits(magnitude), this);
    }

    @Override
    public CurrentPerAngularAcceleration ofBaseUnits(double baseUnitMagnitude) {
        return new ImmutableCurrentPerAngularAcceleration(fromBaseUnits(baseUnitMagnitude), baseUnitMagnitude, this);
    }

    @Override
    public CurrentPerAngularAcceleration zero() {
        return (CurrentPerAngularAcceleration) super.zero();
    }

    @Override
    public CurrentPerAngularAcceleration one() {
        return (CurrentPerAngularAcceleration) super.one();
    }

    @Override
    public MutCurrentPerAngularAcceleration mutable(double initialMagnitude) {
        return new MutCurrentPerAngularAcceleration(initialMagnitude, toBaseUnits(initialMagnitude), this);
    }

    @Override
    public VelocityUnit<CurrentPerAngularAccelerationUnit> per(TimeUnit time) {
        return VelocityUnit.combine(this, time);
    }

    /**
     * Creates a ratio unit between this unit and an arbitrary other unit.
     *
     * @param other the other unit
     * @param <U>   the type of the other unit
     * @return the ratio unit
     */
    public <U extends Unit> PerUnit<CurrentPerAngularAccelerationUnit, U> per(U other) {
        return PerUnit.combine(this, other);
    }

    /**
     * Converts a measurement value in terms of another power unit to this unit.
     *
     * @param magnitude the magnitude of the measurement in terms of the other power unit
     * @param otherUnit the other power unit
     * @return the value of the measurement in terms of this unit
     */
    public double convertFrom(double magnitude, CurrentPerAngularAccelerationUnit otherUnit) {
        return fromBaseUnits(otherUnit.toBaseUnits(magnitude));
    }
}