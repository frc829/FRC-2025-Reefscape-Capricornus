package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Manipulator;

import static digilib.claws.Claw.*;
import static digilib.claws.Claw.Value.*;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class CoralScore {
    private static final double armL1Degrees = 9.0;
    private static final double armL2Degrees = 39;
    private static final double armL3Degrees = 39;
    private static final double armL4Degrees = 46.0;
    private static final double armSafeDegrees = 40.0;
    private static final double armResetDegrees = 90.0;

    private static final double elevatorL1CM = 13.0;
    private static final double elevatorL2CM = 16.0;
    private static final double elevatorL3CM = 36.0;
    private static final double elevatorL4CM = 64.0;
    private static final double elevatorResetCM = 10.0;

    private static final double wristL1Degrees = 90.0;
    private static final double wristSafeDegrees = 0.0;

    private static final Value coralClawScore = OPEN;
    private static final Value coralClawReset = CLOSED;

    private final Manipulator manipulator;
    private final Trigger isArmSafeForWristL1;
    private final Trigger isArmSafeForWrist;


    public CoralScore(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.isArmSafeForWristL1 = manipulator.arm().lte(armSafeDegrees);
        this.isArmSafeForWrist = manipulator.arm().gte(armSafeDegrees);
    }

    public Command l1Align() {
        return sequence(
                parallel(
                        manipulator.elevator().toHeight(elevatorL1CM / 100.0),
                        manipulator.arm().toAngle(armL1Degrees))
                        .until(isArmSafeForWristL1),
                parallel(
                        manipulator.elevator().toHeight(elevatorL1CM / 100.0),
                        manipulator.arm().toAngle(armL1Degrees),
                        manipulator.wrist().toAngle(wristL1Degrees)))
                .withName("Coral Score: L1: Align");
    }

    public Command l2Align() {
        return parallel(
                manipulator.elevator().toHeight(elevatorL2CM / 100.0),
                manipulator.arm().toAngle(armL2Degrees),
                manipulator.wrist().toAngle(wristSafeDegrees))
                .withName("Coral Score: L2: Align");
    }

    public Command l3Align() {
        return parallel(
                manipulator.elevator().toHeight(elevatorL3CM / 100.0),
                manipulator.arm().toAngle(armL3Degrees),
                manipulator.wrist().toAngle(wristSafeDegrees))
                .withName("Coral Score: L3: Align");
    }

    public Command l4Align() {
        return parallel(
                manipulator.elevator().toHeight(elevatorL4CM / 100.0),
                manipulator.arm().toAngle(armL4Degrees),
                manipulator.wrist().toAngle(wristSafeDegrees))
                .withName("Coral Score: L4: Align");
    }

    public Command l1Score() {
//        return manipulator.coralClaw().toClawValue(OPEN);
         return manipulator.coralIntakeWheel().toVelocity(() -> -0.2);
    }

    public Command l234Score() {
        return manipulator.coralClaw().toValue(coralClawScore);
    }

    public Command reset() {
        return sequence(
                manipulator.coralClaw().toValue(coralClawReset),
                parallel(
                        manipulator.elevator().toHeight(elevatorResetCM / 100.0),
                        manipulator.arm().toAngle(armResetDegrees))
                        .until(isArmSafeForWrist),
                parallel(
                        manipulator.elevator().toHeight(elevatorResetCM / 100.0),
                        manipulator.arm().toAngle(armResetDegrees),
                        manipulator.wrist().toAngle(wristSafeDegrees)))
                .withName("Coral Score: Reset");
    }
}
