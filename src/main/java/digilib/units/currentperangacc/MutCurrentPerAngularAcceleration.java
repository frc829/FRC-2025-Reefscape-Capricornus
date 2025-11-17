package digilib.units.currentperangacc;

import edu.wpi.first.units.mutable.MutableMeasureBase;

@SuppressWarnings({"cast", "checkstyle", "PMD"})
public final class MutCurrentPerAngularAcceleration
        extends MutableMeasureBase<CurrentPerAngularAccelerationUnit, CurrentPerAngularAcceleration, MutCurrentPerAngularAcceleration>
        implements CurrentPerAngularAcceleration {
    public MutCurrentPerAngularAcceleration(double magnitude, double baseUnitMagnitude, CurrentPerAngularAccelerationUnit unit) {
        super(magnitude, baseUnitMagnitude, unit);
    }

    @Override
    public CurrentPerAngularAcceleration copy() {
        return new ImmutableCurrentPerAngularAcceleration(magnitude(), baseUnitMagnitude(), unit());
    }
}