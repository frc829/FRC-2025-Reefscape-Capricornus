package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

        return Commands.none();

    }
}
