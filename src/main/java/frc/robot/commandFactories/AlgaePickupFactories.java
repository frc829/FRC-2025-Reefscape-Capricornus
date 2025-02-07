package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class AlgaePickupFactories {
    private final SubsystemCommandFactories factories;

    public AlgaePickupFactories(SubsystemCommandFactories factories) {
        this.factories = factories;
    }

    private Command createAlgaePickup(
            String name,
            Distance elevatorHeight,
            Angle armAngle,
            Angle wristAngle,
            DoubleSupplier intake0Velocity,
            DoubleSupplier intake1Velocity) {

        return factories.coral.open()
                .andThen(factories.algae.open())
                .andThen(factories.wrist.goToAngle(wristAngle)
                        .until(factories.wrist.atPosition(Degrees.of(0.0), Degrees.of(0.1)))
                        .asProxy())
                .andThen(
                        factories.arm.goToAngle(armAngle)
                                .until(factories.arm.atPosition(armAngle, Degrees.of(1.000)))
                                .asProxy()
                                .alongWith(factories.elevator.goToPosition(elevatorHeight)
                                        .until(factories.elevator.atPosition(elevatorHeight, Meters.of(0.001)))
                                        .asProxy()))
                .andThen(factories.intake.moveAtVelocity(intake0Velocity, intake1Velocity))
                .withName(name);
    }

    public Command createCoralPickup(
            String name,
            Distance elevatorHeight,
            Angle armAngle,
            Angle wristAngle,
            LinearVelocity intake0Velocity,
            LinearVelocity intake1Velocity) {
        return null;
    }


}
