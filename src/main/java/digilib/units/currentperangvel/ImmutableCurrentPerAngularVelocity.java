package digilib.units.currentperangvel;

import edu.wpi.first.units.Measure;

@SuppressWarnings({"cast", "checkstyle", "PMD"})
public record ImmutableCurrentPerAngularVelocity(double magnitude, double baseUnitMagnitude, CurrentPerAngularVelocityUnit unit) implements CurrentPerAngularVelocity {
    @Override
    public CurrentPerAngularVelocity copy() {
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