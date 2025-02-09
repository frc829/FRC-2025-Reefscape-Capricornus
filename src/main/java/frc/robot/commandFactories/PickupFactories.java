package frc.robot.commandFactories;

import digilib.claws.ClawState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

import static digilib.claws.ClawState.ClawValue.*;
import static edu.wpi.first.units.Units.*;

public class PickupFactories {
    public final SubsystemCommandFactories factories;

    public PickupFactories(SubsystemCommandFactories factories) {
        this.factories = factories;
    }

    public Command algaeFloor() {
        return createPickup(
                "Pickup:Algae Floor",
                OPEN,
                CLOSED,
                Meters.of(0.4),
                Degrees.of(180),
                Value.of(0.5),
                Value.of(0.5));
    }

    public Command algaeL2() {
        return createPickup(
                "Pickup:Algae L2",
                OPEN,
                CLOSED,
                Meters.of(1.5),
                Degrees.of(35),
                Value.of(0.5),
                Value.of(0.5)
        );
    }

    public Command algaeL3() {
        return createPickup(
                "Pickup:Algae L3",
                OPEN,
                CLOSED,
                Meters.of(1.75),
                Degrees.of(35),
                Value.of(0.5),
                Value.of(0.5)
        );
    }

    public Command coralFloor() {
        return createPickup(
                "Pickup:Coral Floor",
                CLOSED,
                CLOSED,
                Meters.of(0.4),
                Degrees.of(-40),
                Value.of(0.5),
                Value.of(0.5));
    }

    public Command coralStationFront() {
        return createPickup(
                "Pickup:Coral Station Front",
                CLOSED,
                CLOSED,
                Meters.of(1.0),
                Degrees.of(0.0),
                Value.of(0.5),
                Value.of(0.5));
    }

    public Command coralStationBack() {
        return createPickup(
                "Pickup:Coral Station Back",
                CLOSED,
                CLOSED,
                Meters.of(1.0),
                Degrees.of(180.0),
                Value.of(0.5),
                Value.of(0.5));
    }

    private Command createPickup(
            String name,
            ClawState.ClawValue algaeClawValue,
            ClawState.ClawValue coralClawValue,
            Distance elevatorHeight,
            Angle armAngle,
            Dimensionless intake0Velocity,
            Dimensionless intake1Velocity) {

        return factories.algae.setClawValue(algaeClawValue)
                .andThen(factories.coral.setClawValue(coralClawValue))
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
