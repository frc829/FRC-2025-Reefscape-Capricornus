package digilib.units.currentperangacc;

import edu.wpi.first.units.Measure;

@SuppressWarnings({"cast", "checkstyle", "PMD"})
public record ImmutableCurrentPerAngularAcceleration(double magnitude, double baseUnitMagnitude, CurrentPerAngularAccelerationUnit unit) implements CurrentPerAngularAcceleration {
    @Override
    public CurrentPerAngularAcceleration copy() {
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