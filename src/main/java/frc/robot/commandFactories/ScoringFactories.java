package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;

public class ScoringFactories {

    private final ManipulatorFactories factories;

    public ScoringFactories(ManipulatorFactories factories) {
        this.factories = factories;
    }

    public Command l1Align() {
        return Commands.none();
    }

    public Command l2Align() {
        return Commands.none();
    }

    public Command l3Align() {
        return Commands.none();
    }

    public Command l4Align() {
        return Commands.none();
    }

    public Command coralScore() {
        return Commands.none();
    }

    public Command bargeAlign() {
        return Commands.none();
    }

    public Command processorAlign() {
        return Commands.none();
    }

    public Command bargeScore() {
        return Commands.none();
    }

    public Command processorScore() {
        return Commands.none();
    }

    public Command climb(Supplier<Dimensionless> dutyCycle) {
        return factories.climbAtDutyCycle(dutyCycle);
    }

}
