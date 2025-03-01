package frc.robot.commands.game;

import digilib.claws.ClawValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.system.Manipulator;

import static digilib.claws.ClawValue.CLOSED;
import static digilib.claws.ClawValue.OPEN;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AlgaePickup {
    private static final Angle armFloor = Degrees.of(-43.7);
    private static final Angle armL2 = Degrees.of(-7.0);
    private static final Angle armL3 = Degrees.of(-7.0);
    private static final Angle armHold = Degrees.of(0.0);

    private static final Distance elevatorFloor = Centimeters.of(17.0);
    private static final Distance elevatorL2 = Centimeters.of(27.0);
    private static final Distance elevatorL3 = Centimeters.of(51.0);
    private static final Distance elevatorHold = Centimeters.of(0.0);

    private static final Angle wristPickup = Degrees.of(90.0);

    private static final ClawValue algaeClawValue = CLOSED;
    private static final ClawValue coralClawValue = OPEN;

    private static final Dimensionless algaeSpeed = Percent.of(-100);
    private static final Dimensionless coralSpeed = Percent.of(-100);

    private static final Angle armSafeDown = Degrees.of(0.0);

    private final Manipulator manipulator;
    private final Trigger hasAlgae;
    private final Trigger isArmSafeForWristDown;

    public AlgaePickup(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.hasAlgae = manipulator.hasAlgae;
        this.isArmSafeForWristDown = manipulator.armLessThan(armSafeDown);
    }

    public Command floor() {
        return parallel(
                elevatorAlgaeFloor(),
                armAlgaeFloor(),
                either(
                        idle(),
                        manipulator.wristTo(wristPickup),
                        isArmSafeForWristDown),
                either(
                        idle(),
                        algaeIntake().until(hasAlgae),
                        isArmSafeForWristDown),
                either(
                        idle(),
                        claws(),
                        isArmSafeForWristDown))
                .withName("Algae Pickup: Floor");
    }

    public Command oldL2() {
        return parallel(
                elevatorAlgaeL2(),
                armAlgaeL2(),
                either(
                        idle(),
                        manipulator.wristTo(wristPickup),
                        isArmSafeForWristDown),
                either(
                        idle(),
                        algaeIntake().until(hasAlgae),
                        isArmSafeForWristDown),
                either(
                        idle(),
                        claws(),
                        isArmSafeForWristDown))
                .withName("Algae Pickup: L2");
    }

    public Command L2() {
        return sequence(
                parallel(elevatorAlgaeL2(),
                        armAlgaeL2())
                        .until(isArmSafeForWristDown),
                parallel(elevatorAlgaeL2(),
                        armAlgaeL2(),
                        claws(),
                        manipulator.wristTo(wristPickup),
                        algaeIntake().asProxy().until(hasAlgae)))
                .withName("Algae Pickup: L2 ");

    }

    public Command oldL3() {
        return parallel(
                elevatorAlgaeL3(),
                armAlgaeL3(),
                either(
                        idle(),
                        manipulator.wristTo(wristPickup),
                        isArmSafeForWristDown),
                either(
                        idle(),
                        algaeIntake().until(hasAlgae),
                        isArmSafeForWristDown),
                either(
                        idle(),
                        claws(),
                        isArmSafeForWristDown))
                .withName("Algae Pickup: L3");
    }

    public Command L3() {
        return sequence(
                parallel(elevatorAlgaeL3(),
                        armAlgaeL3())
                        .until(isArmSafeForWristDown),
                parallel(elevatorAlgaeL3(),
                        armAlgaeL3(),
                        claws(),
                        manipulator.wristTo(wristPickup),
                        algaeIntake().asProxy().until(hasAlgae)))
                .withName("Algae Pickup: L3 ");

    }

    public Command hold() {
        return parallel(
                elevatorAlgaeHold(),
                armAlgaeHold())
                .withName("Algae Hold");
    }

    private Command algaeIntake() {
        return sequence(
                race(
                        manipulator.intakeToSpeed(algaeSpeed, coralSpeed),
                        waitSeconds(0.5)
                ),
                manipulator.intakeToSpeed(algaeSpeed, coralSpeed)
                        .until(hasAlgae)
        );
    }

    private Command claws() {
        return parallel(
                manipulator.setAlgaeClaw(algaeClawValue),
                manipulator.setCoralClaw(coralClawValue));
    }

    private Command elevatorAlgaeL2() {
        return manipulator.elevatorTo(elevatorL2);
    }

    private Command elevatorAlgaeL3() {
        return manipulator.elevatorTo(elevatorL3);
    }

    private Command elevatorAlgaeHold() {
        return manipulator.elevatorTo(elevatorHold);
    }

    private Command elevatorAlgaeFloor() {
        return manipulator.elevatorTo(elevatorFloor);
    }

    private Command armAlgaeL2() {
        return manipulator.armTo(armL2);
    }

    private Command armAlgaeL3() {
        return manipulator.armTo(armL3);
    }

    private Command armAlgaeHold() {
        return manipulator.armTo(armHold);
    }

    private Command armAlgaeFloor() {
        return manipulator.armTo(armFloor);
    }
}
