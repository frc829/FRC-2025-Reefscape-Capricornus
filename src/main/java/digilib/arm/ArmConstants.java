package digilib.arm;

/**
 * @param name                      The name of the arm in reference to the robot
 * @param reduction                 The ratio of output to input.
 * @param startingAngleDegrees      The starting angle for the arm simulation
 * @param minAngleDegrees           The minimum angle in arm's range of motion.
 * @param maxAngleDegrees           The maximum angle in the arm's range of motion.
 * @param ksVolts                   The static friction gain in Volts.
 * @param kgVolts                   The gravity gain in Volts.
 * @param kvVoltsPerRPS             The velocity gain.
 * @param kaVoltsPerRPSSquared      The acceleration gain.
 * @param maxVelocityRPS            The maximum possible velocity of the arm.
 * @param maxAccelerationRPSSquared The maximum possible acceleration of the arm.
 */
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
