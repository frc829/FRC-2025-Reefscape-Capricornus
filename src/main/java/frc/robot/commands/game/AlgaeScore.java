package frc.robot.commands.game;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.system.Manipulator;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class AlgaeScore {
    private static final Distance elevatorAlgaeProc = Centimeters.of(3.0);
    private static final Angle armAlgaeProc = Degrees.of(-20.0);


    private static final Distance elevatorBargeTravel = Centimeters.of(45.0);
    private static final Distance armSafe = Centimeters.of(40.0);
    private static final Distance elevatorBarge = Centimeters.of(65.0);
    private static final Angle armBargeTravel = Degrees.of(30.0);
    private static final Angle armBarge = Degrees.of(48.0);


    private final Manipulator manipulator;
    private final Trigger isArmSafe;


    public AlgaeScore(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.isArmSafe = manipulator.elevatorGreaterThan(armSafe);
    }

    public Command bargeAlign() {
        return sequence(
                parallel(
                        manipulator.elevatorTo(elevatorBargeTravel),
                        manipulator.armTo(armBargeTravel))
                        .until(isArmSafe),
                parallel(manipulator.elevatorTo(elevatorBarge),
                        manipulator.armTo(armBarge)))
                .withName("Algae Score: Barge: Align");
    }

    public Command bargeScore() {
        return manipulator.intakeToSpeed(Percent.of(100.0), Percent.of(100.0))
                .withName("Algae Score: Barge");
    }

    public Command bargeScoreReset() {
        return parallel(
                manipulator.elevatorTo(Centimeters.of(0.0)),
                manipulator.armTo(Degrees.of(90.0)),
                manipulator.wristTo(Degrees.of(0.0)))
                .withName("Algae Score: Barge: Reset");
    }

    public Command processorAlign() {
        return parallel(
                manipulator.elevatorTo(elevatorAlgaeProc),
                manipulator.armTo(armAlgaeProc))
                .withName("Algae Score: Processor: Align");

    }

    public Command processorScore() {
        return manipulator.intakeToSpeed(Percent.of(100.0), Percent.of(100.0))
                .withName("Algae Score: Processor");
    }
}
