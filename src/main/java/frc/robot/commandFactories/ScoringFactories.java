package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static digilib.claws.ClawState.ClawValue.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class ScoringFactories {

    private final SubsystemCommandFactories factories;
    private final ResetFactories reset;
    private final Angle safeArmAngleForWrist = Degrees.of(10.0);

    public ScoringFactories(SubsystemCommandFactories factories,
                            ResetFactories reset) {
        this.factories = factories;
        this.reset = reset;
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

    private Command createCoralAlign(
            String name,
            Distance elevatorHeight,
            Angle armAngle,
            Angle wristAngle) {
        return factories.algae.setClawValue(CLOSED)
                .andThen(factories.coral.setClawValue(CLOSED))
                .andThen(Commands.either(
                        createCoralAlignSafe(elevatorHeight, armAngle, wristAngle),
                        createCoralAlignUnSafe(elevatorHeight, armAngle, wristAngle),
                        factories.arm.lessThanPosition(safeArmAngleForWrist, Degrees.of(0.1))))
                .withName(name);

    }

    private Command createCoralAlignSafe(
            Distance elevatorHeight,
            Angle armAngle,
            Angle wristAngle) {
        return factories.wrist.goToAngle(wristAngle).asProxy()
                .alongWith(factories.elevator.goToPosition(elevatorHeight).asProxy())
                .until(factories.wrist.atPosition(wristAngle, Degrees.of(0.1)))
                .andThen(factories.elevator
                        .goToPosition(elevatorHeight).asProxy()
                        .until(factories.elevator.atPosition(elevatorHeight, Meters.of(0.001)))
                        .alongWith(factories.arm
                                .goToAngle(armAngle).asProxy()
                                .until(factories.arm.atPosition(armAngle, Degrees.of(0.1)))));
    }

    private Command createCoralAlignUnSafe(
            Distance elevatorHeight,
            Angle armAngle,
            Angle wristAngle) {
        return factories.wrist.goToAngle(Degrees.of(0.0)).asProxy()
                .alongWith(factories.elevator.goToPosition(elevatorHeight).asProxy())
                .until(factories.wrist.atPosition(Degrees.of(0.0), Degrees.of(0.1)))
                .andThen(factories.elevator
                        .goToPosition(elevatorHeight).asProxy()
                        .alongWith(factories.arm.goToAngle(armAngle)));
    }
}
