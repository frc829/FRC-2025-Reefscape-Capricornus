package digilib.wrist;

public record WristConstants(String name,
                             double reduction,
                             double startingAngleDegrees,
                             double minAngleDegrees,
                             double maxAngleDegrees,
                             double maxControlVoltage,
                             double ksVolts,
                             double kvVoltsPerRPS,
                             double kaVoltsPerRPSSquared,
                             double maxVelocityRPS,
                             double maxAccelerationRPSSquared) {
}
