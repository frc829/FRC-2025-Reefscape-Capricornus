package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class PickupFactories {
    public final SubsystemCommandFactories factories;

    public PickupFactories(SubsystemCommandFactories factories) {
        this.factories = factories;
    }

    public Command algaeFloor() {
        return createPickup(
                "Pickup:Algae Floor",
                Meters.of(0.4),
                Degrees.of(180),
                0.5,
                0.5);
    }

    public Command algaeL2() {
        return createPickup(
                "Pickup:Algae L2",
                Meters.of(1.5),
                Degrees.of(35),
                0.5,
                0.5);
    }

    public Command algaeL3() {
        return createPickup(
                "Pickup:Algae L3",
                Meters.of(1.75),
                Degrees.of(35),
                0.5,
                0.5);
    }

    public Command coralFloor() {
        return createPickup(
                "Pickup:Coral Floor",
                Meters.of(0.4),
                Degrees.of(-40),
                0.5,
                0.5);
    }

    public Command coralStationFront() {
        return createPickup(
                "Pickup:Coral Station Front",
                Meters.of(1.0),
                Degrees.of(0.0),
                0.5,
                0.5);
    }

    public Command coralStationBack() {
        return createPickup(
                "Pickup:Coral Station Back",
                Meters.of(1.0),
                Degrees.of(180.0),
                0.5,
                0.5);
    }

    private Command createPickup(
            String name,
            Distance elevatorHeight,
            Angle armAngle,
            double intake0Velocity,
            double intake1Velocity) {

        return factories.algae.close()
                .andThen(factories.coral.close())
                .andThen(factories.wrist.goToAngle(Degrees.of(90.0))
                        .until(factories.wrist.atPosition(Degrees.of(90.0), Degrees.of(0.1)))
                        .asProxy())
                .andThen(factories.arm.goToAngle(armAngle)
                        .until(factories.arm.atPosition(armAngle, Degrees.of(1.000)))
                        .asProxy()
                        .alongWith(factories.elevator.goToPosition(elevatorHeight)
                                .until(factories.elevator.atPosition(elevatorHeight, Meters.of(0.001)))
                                .asProxy()))
                .andThen(factories.intake.moveAtVelocity(() -> intake0Velocity, () -> intake1Velocity))
                .withName(name);
    }
}
