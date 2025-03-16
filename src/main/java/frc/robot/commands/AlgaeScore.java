package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Manipulator;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class AlgaeScore {
    private static final double armSafeDegrees = 40.0;
    private static final double armBargeTravelDegrees = 30.0;
    private static final double armBargeDegrees = 48.0;
    private static final double armResetDegrees = 90.0;


    private static final double elevatorBargeTravelCM = 45.0;
    private static final double elevatorBargeCM = 65.0;
    private static final double elevatorResetCM = 10.0;

    private static final double wristSafeDegrees = 0.0;

    private static final double algaeSpeed = 1;
    private static final double coralSpeed = 1;

    private final Manipulator manipulator;
    private final Trigger isArmSafe;

    public AlgaeScore(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.isArmSafe = manipulator.arm().gte(armSafeDegrees);
    }

    public Command bargeAlign() {
        return sequence(
                parallel(
                        manipulator.elevator().toHeight(elevatorBargeTravelCM / 100.0),
                        manipulator.arm().toAngle(armBargeTravelDegrees))
                        .until(isArmSafe),
                parallel(manipulator.elevator().toHeight(elevatorBargeCM / 100.0),
                        manipulator.arm().toAngle(armBargeDegrees)))
                .withName("Algae Score: Barge: Align");
    }

    public Command score() {
        return parallel(
                manipulator.algaeIntakeWheel().toVelocity(() -> algaeSpeed),
                manipulator.coralIntakeWheel().toVelocity(() -> coralSpeed))
                .withName("Algae Score");
    }

    public Command reset() {
        return sequence(
                parallel(
                        manipulator.elevator().toHeight(elevatorResetCM / 100.0),
                        manipulator.arm().toAngle(armResetDegrees))
                        .until(isArmSafe),
                parallel(
                        manipulator.elevator().toHeight(elevatorResetCM / 100.0),
                        manipulator.arm().toAngle(armResetDegrees),
                        manipulator.wrist().toAngle(wristSafeDegrees)))
                .withName("Algae Score: Reset");
    }
}
