package frc.robot.commandFactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Degrees;

public class PickupFactories {
    public final SubsystemCommandFactories factories;

    public PickupFactories(SubsystemCommandFactories factories) {
        this.factories = factories;
    }

    public Command algaeFloor() {
        return Commands.none();

    }

    public Command algaeL2() {
        return Commands.none();

    }

    public Command algaeL3() {
        return Commands.none();

    }

    public Command coralFloor() {
        return Commands.none();

    }

    public Command coralStationFront() {
        return factories.wrist.goToAngle(Degrees.of(90.0))
                .until(factories.wrist.atPosition(Degrees.of(90), Degrees.of(1))).asProxy()
                .withName("Coral Station Front");

    }

    public Command coralStationBack() {
        return Commands.none();
    }
}
