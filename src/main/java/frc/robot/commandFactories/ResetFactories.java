package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.*;

public class ResetFactories {
    public final SubsystemCommandFactories factories;

    public ResetFactories(SubsystemCommandFactories factories) {
        this.factories = factories;
    }

    public Command algae() {
        return createReset(
                "Reset:Holding Algae",
                Meters.of(0.4),
                Degrees.of(0.0),
                Degrees.of(0.0));
    }

    public Command coral() {
        return createReset(
                "Reset:Holding Coral",
                Meters.of(0.4),
                Degrees.of(90.0),
                Degrees.of(0.0));
    }

    private Command createReset(
            String name,
            Distance elevatorHeight,
            Angle armAngle,
            Angle wristAngle) {

        return factories.wrist.goToAngle(wristAngle)
                .until(factories.wrist.atPosition(Degrees.of(0.0), Degrees.of(0.1)))
                .asProxy()
                .andThen(factories.arm.goToAngle(armAngle)
                        .until(factories.arm.atPosition(armAngle, Degrees.of(1.000)))
                        .asProxy()
                        .alongWith(factories.elevator.goToPosition(elevatorHeight)
                                .until(factories.elevator.atPosition(elevatorHeight, Meters.of(0.001)))
                                .asProxy()))
                .withName(name);
    }
}
