package digilib.units.voltsperangacc;

import edu.wpi.first.units.Measure;

@SuppressWarnings({"cast", "checkstyle", "PMD"})
public record ImmutableVoltsPerAngularAcceleration(double magnitude, double baseUnitMagnitude, VoltsPerAngularAccelerationUnit unit) implements VoltsPerAngularAcceleration {
    @Override
    public VoltsPerAngularAcceleration copy() {
        return this;
    }

    @Override
    public String toString() {
        return toShortString();
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof Measure<?> m && isEquivalent(m);
    }
}