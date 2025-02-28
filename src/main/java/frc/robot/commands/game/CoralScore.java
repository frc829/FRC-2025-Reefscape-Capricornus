package frc.robot.commands.game;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.system.Manipulator;

import static digilib.claws.ClawValue.CLOSED;
import static digilib.claws.ClawValue.OPEN;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class CoralScore {

    private static final Distance elevatorL1 = Centimeters.of(5.0);
    private static final Angle armL1 = Degrees.of(35.0);
    private static final Angle armSafe = Degrees.of(40.0);

    private static final Distance elevatorL2 = Centimeters.of(19.0);
    private static final Angle armL2 = Degrees.of(42.6);

    private static final Distance elevatorL3 = Centimeters.of(40.0);
    private static final Angle armL3 = Degrees.of(42.6);

    private static final Distance elevatorL4 = Centimeters.of(65.0);
    private static final Angle armL4 = Degrees.of(48.0);

    private static final Angle wristL1 = Degrees.of(90.0);
    private static final Angle wristL234 = Degrees.of(0.0);


    private final Manipulator manipulator;
    private final Trigger isArmSafeForWrist;


    public CoralScore(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.isArmSafeForWrist = manipulator.armLessThan(armSafe);
    }

    public Command l1Align() {
        return sequence(
                parallel(
                        manipulator.elevatorTo(elevatorL1),
                        manipulator.armTo(armL1))
                        .until(isArmSafeForWrist),
                parallel(
                        manipulator.elevatorTo(elevatorL1),
                        manipulator.armTo(armL1),
                        manipulator.wristTo(wristL1)))
                .withName("Coral Score: L1: Align");
    }

    public Command l2Align() {
        return parallel(
                manipulator.elevatorTo(elevatorL2),
                manipulator.armTo(armL2),
                manipulator.wristTo(wristL234))
                .withName("Coral Score: L2: Align");
    }

    public Command l3Align() {
        return parallel(
                manipulator.elevatorTo(elevatorL3),
                manipulator.armTo(armL3),
                manipulator.wristTo(wristL234))
                .withName("Coral Score: L3: Align");
    }

    public Command l4Align() {
        return parallel(
                manipulator.elevatorTo(elevatorL4),
                manipulator.armTo(armL4),
                manipulator.wristTo(wristL234))
                .withName("Coral Score: L4: Align");
    }

    public Command l1Score() {
        return manipulator.intakeToSpeed(Percent.of(0.0), Percent.of(-20.0));
    }

    public Command l234Score() {
        return manipulator.setCoralClaw(OPEN);
    }

    public Command L234ScoreReset() {
        return manipulator.setCoralClaw(CLOSED);
    }
}
