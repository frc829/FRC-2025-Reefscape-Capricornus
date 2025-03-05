package digilib.arm;

public record ArmConstants(String name,
                           double reduction,
                           double startingAngleDegrees,
                           double minAngleDegrees,
                           double maxAngleDegrees,
                           double ksVolts,
                           double kgVolts,
                           double kvVoltsPerRPS,
                           double kaVoltsPerRPSSquared,
                           double maxVelocityRPS,
                           double maxAccelerationRPSSquared) {
}
