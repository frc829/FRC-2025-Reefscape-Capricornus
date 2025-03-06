package digilib.climber;

public record ClimberConstants(String name,
                               double reduction,
                               double drumRadiusMeters,
                               double startingLengthMeters,
                               double minLengthMeters,
                               double maxLengthMeters,
                               double maxControlVoltage,
                               double ksVolts,
                               double kvVoltsPerMPS,
                               double kaVoltsPerMPSSquared,
                               double maxVelocityMPS,
                               double maxAccelerationMPSSquared) {
}
