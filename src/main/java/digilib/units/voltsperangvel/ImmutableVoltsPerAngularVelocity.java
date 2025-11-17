package digilib.units.voltsperangvel;

import edu.wpi.first.units.*;

@SuppressWarnings({"cast", "checkstyle", "PMD"})
public record ImmutableVoltsPerAngularVelocity(double magnitude, double baseUnitMagnitude, VoltsPerAngularVelocityUnit unit) implements VoltsPerAngularVelocity {
    @Override
    public VoltsPerAngularVelocity copy() {
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