package frc.robot.commands.game;

import digilib.claws.ClawValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.system.Manipulator;

import static digilib.claws.ClawValue.*;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class CoralPickup {

    private static final Angle armFloor = Degrees.of(-43.7);
    private static final Angle armStation = Degrees.of(50.0);
    private static final Angle armHold = Degrees.of(90.0);

    private static final Distance elevatorFloor = Centimeters.of(17.0);
    private static final Distance elevatorStation = Centimeters.of(14.0);
    private static final Distance elevatorHold = Centimeters.of(1.0);

    private static final Angle wristPickup = Degrees.of(90.0);
    private static final Angle wristSafe = Degrees.of(0.0);

    private static final ClawValue algaeClawValue = OPEN;
    private static final ClawValue coralClawValue = CLOSED;

    private static final Dimensionless algaeSpeed = Percent.of(0);
    private static final Dimensionless coralSpeed = Percent.of(100);

    private static final Angle armSafeDown = Degrees.of(60.0);
    private static final Angle armSafeUp = Degrees.of(0.0);

    private final Manipulator manipulator;
    public final Trigger hasCoral;
    private final Trigger isArmSafeForWristDown;
    private final Trigger isArmSafeForWristUp;


    public CoralPickup(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.hasCoral = manipulator.hasCoral;
        this.isArmSafeForWristDown = manipulator.armLessThan(armSafeDown);
        this.isArmSafeForWristUp = manipulator.armGreaterThan(armSafeUp);
    }

    public Command floor() {
        return sequence(
                parallel(elevatorCoralFloor(),
                        armCoralFloor())
                        .until(isArmSafeForWristDown),
                parallel(elevatorCoralFloor(),
                        armCoralFloor(),
                        clawPickup(),
                        manipulator.wristTo(wristPickup),
                        coralIntake()))
                .withName("Coral Pickup: Floor");
    }

    public Command station() {
        return sequence(
                parallel(elevatorStation(),
                        armCoralStation())
                        .until(isArmSafeForWristDown),
                parallel(elevatorStation(),
                        armCoralStation(),
                        clawPickup(),
                        manipulator.wristTo(wristPickup),
                        coralIntake().until(hasCoral).asProxy())
                )
                .withName("Coral Pickup: Station ");
    }

    public Command hold() {
        return sequence(
                parallel(armCoralHold(), coralHoldIntake())
                        .until(manipulator.armGreaterThan(Degrees.of(-20))),
                parallel(elevatorCoralHold(), armCoralHold(), coralHoldIntake())
                        .until(isArmSafeForWristUp),
                parallel(elevatorCoralHold(),
                        armCoralHold(),
                        clawHold(),
                        coralHoldIntake(),
                        manipulator.wristTo(wristSafe)))
                .withName("Coral Hold");
    }

    private Command coralIntake() {
        return manipulator.intakeToSpeed(algaeSpeed, coralSpeed).until(hasCoral)
                .andThen(manipulator.intakeToSpeed(Percent.of(0.0), Percent.of(10.0)));
    }

    private Command coralHoldIntake() {
        return manipulator.intakeToSpeed(Percent.of(0.0), Percent.of(10.0));
    }

    private Command clawPickup() {
        return parallel(
                manipulator.setAlgaeClaw(algaeClawValue),
                manipulator.setCoralClaw(coralClawValue));
    }

    private Command clawHold() {
        return parallel(
                manipulator.setAlgaeClaw(algaeClawValue));
    }

    private Command elevatorStation() {
        return manipulator.elevatorTo(elevatorStation);
    }

    private Command elevatorCoralFloor() {
        return manipulator.elevatorTo(elevatorFloor);
    }

    private Command elevatorCoralHold() {
        return manipulator.elevatorTo(elevatorHold);
    }

    private Command armCoralStation() {
        return manipulator.armTo(armStation);
    }

    private Command armCoralFloor() {
        return manipulator.armTo(armFloor);
    }

    private Command armCoralHold() {
        return manipulator.armTo(armHold);
    }
}
