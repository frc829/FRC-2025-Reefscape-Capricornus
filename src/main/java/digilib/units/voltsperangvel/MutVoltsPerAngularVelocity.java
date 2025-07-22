package digilib.units.voltsperangvel;

import edu.wpi.first.units.mutable.MutableMeasureBase;

@SuppressWarnings({"cast", "checkstyle", "PMD"})
public final class MutVoltsPerAngularVelocity
        extends MutableMeasureBase<VoltsPerAngularVelocityUnit, VoltsPerAngularVelocity, MutVoltsPerAngularVelocity>
        implements VoltsPerAngularVelocity {
    public MutVoltsPerAngularVelocity(double magnitude, double baseUnitMagnitude, VoltsPerAngularVelocityUnit unit) {
        super(magnitude, baseUnitMagnitude, unit);
    }

    @Override
    public VoltsPerAngularVelocity copy() {
        return new ImmutableVoltsPerAngularVelocity(magnitude(), baseUnitMagnitude(), unit());
    }
}