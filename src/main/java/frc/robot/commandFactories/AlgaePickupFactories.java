package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;

public class AlgaePickupFactories {
    private final SubsystemCommandFactories factories;

    public AlgaePickupFactories(SubsystemCommandFactories factories) {
        this.factories = factories;
    }

    private Command createAlgaePickup(
            Distance elevatorHeight,
            Angle armAngle,
            Angle wristAngle,
            LinearVelocity intake0Velocity,
            LinearVelocity intake1Velocity) {

        return factories.algae.close()
                .andThen(factories.wrist.goToAngle(wristAngle))
                .until(factories.wrist.atPosition(Degrees.of(0.0), Degrees.of(0.1)));

    }


}
