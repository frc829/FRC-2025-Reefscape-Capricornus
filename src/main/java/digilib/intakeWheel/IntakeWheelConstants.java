package digilib.intakeWheel;

public record IntakeWheelConstants(String name,
                                   double reduction,
                                   double maxControlVoltage,
                                   double ksVolts,
                                   double kvVoltsPerRPS,
                                   double kaVoltsPerRPSSquared,
                                   double maxVelocityRPS,
                                   double maxAccelerationRPSSquared) {
}
