package digilib.units.voltsperangacc;

import edu.wpi.first.units.mutable.MutableMeasureBase;

@SuppressWarnings({"cast", "checkstyle", "PMD"})
public final class MutVoltsPerAngularAcceleration
        extends MutableMeasureBase<VoltsPerAngularAccelerationUnit, VoltsPerAngularAcceleration, MutVoltsPerAngularAcceleration>
        implements VoltsPerAngularAcceleration {
    public MutVoltsPerAngularAcceleration(double magnitude, double baseUnitMagnitude, VoltsPerAngularAccelerationUnit unit) {
        super(magnitude, baseUnitMagnitude, unit);
    }

    @Override
    public VoltsPerAngularAcceleration copy() {
        return new ImmutableVoltsPerAngularAcceleration(magnitude(), baseUnitMagnitude(), unit());
    }
}