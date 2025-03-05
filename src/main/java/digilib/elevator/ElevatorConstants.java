package digilib.elevator;

public record ElevatorConstants(String name,
                                double  reduction,
                                double drumRadiusMeters,
                                double startingHeightMeters,
                                double minHeightMeters,
                                double maxHeightMeters,
                                double ksVolts,
                                double kgVolts,
                                double kvVoltsPerMPS,
                                double kaVoltsPerMPSSquared,
                                double maxVelocityMPS,
                                double maxAccelerationMPSSquared) {
}
