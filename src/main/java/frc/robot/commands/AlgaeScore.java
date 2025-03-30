package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Manipulator;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class AlgaeScore {
    private static final double armBargeTravelDegrees = 0.0;
    private static final double armBargeDegrees = 57.0;

    private static final double elevatorBargeTravelCM = 49.0;
    private static final double elevatorBargeSafeCM = 47.0;
    private static final double elevatorBargeCM = 67.0;

    private static final double coralSpeed = 1;

    private final Manipulator manipulator;
    private final Trigger isSafe;

    public AlgaeScore(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.isSafe = manipulator.elevator().gte(elevatorBargeSafeCM / 100.0);
    }

    public Command bargeAlign() {
        return sequence(
                parallel(
                        manipulator.elevator().toHeight(elevatorBargeTravelCM / 100.0),
                        manipulator.arm().toAngle(armBargeTravelDegrees))
                        .until(isSafe),
                parallel(manipulator.elevator().toHeight(elevatorBargeCM / 100.0),
                        manipulator.arm().toAngle(armBargeDegrees)))
                .withName("Algae Score: Barge: Align");
    }

    public Command score() {
        return parallel(
                manipulator.coralIntakeWheel().toVelocity(() -> coralSpeed))
                .withName("Algae Score");
    }
}
