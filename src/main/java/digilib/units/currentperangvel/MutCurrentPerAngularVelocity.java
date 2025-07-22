package digilib.units.currentperangvel;

import edu.wpi.first.units.mutable.MutableMeasureBase;

@SuppressWarnings({"cast", "checkstyle", "PMD"})
public final class MutCurrentPerAngularVelocity
        extends MutableMeasureBase<CurrentPerAngularVelocityUnit, CurrentPerAngularVelocity, MutCurrentPerAngularVelocity>
        implements CurrentPerAngularVelocity {
    public MutCurrentPerAngularVelocity(double magnitude, double baseUnitMagnitude, CurrentPerAngularVelocityUnit unit) {
        super(magnitude, baseUnitMagnitude, unit);
    }

    @Override
    public CurrentPerAngularVelocity copy() {
        return new ImmutableCurrentPerAngularVelocity(magnitude(), baseUnitMagnitude(), unit());
    }
}