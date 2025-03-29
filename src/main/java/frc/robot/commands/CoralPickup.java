package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Manipulator;

import static digilib.claws.Claw.*;
import static digilib.claws.Claw.Value.CLOSED;
import static digilib.claws.Claw.Value.OPEN;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class CoralPickup {

    private static final double armFloorDegrees = -39;
    private static final double armStationDegrees = 50.0;
    private static final double armStationDegreesBack = 134.4;

    private static final double armHoldDegrees = 88.0;
    private static final double armSafeDownDegrees = 60.0;
    private static final double armSafeDownDegreesBack = 120.0;

    private static final double armSafeUpDegrees = 0.0;
    private static final double armSafeUpDegreesBack = 180.0;
    private static final double armSafeElevatorDegrees = -20.0;

    private static final double elevatorFloorCM = 16.5;
    private static final double elevatorStationCM = 13.0;
    private static final double elevatorStationBackCM = 17.2;
    private static final double elevatorHoldCM = 10.0;

    private static final double wristPickupDegrees = 90.0;
    private static final double wristPickupDegreesBack = -90.0;
    private static final double wristSafeDegrees = 0.0;

    private static final double coralSpeed = 1;

    private static final double coralHoldSpeed = 0.05;

    private static final Value algaeClawIntakeValue = OPEN;
    private static final Value algaeClawHoldValue = OPEN;
    private static final Value coralClawValue = CLOSED;

    private final Manipulator manipulator;
    public final Trigger hasCoral;
    private final Trigger isArmSafeForElevatorUp;
    private final Trigger isArmSafeForWristDown;
    private final Trigger isArmSafeForWristUp;
    private final Trigger isArmSafeForWristDownBack;
    private final Trigger isArmSafeForWristUpBack;

    public CoralPickup(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.hasCoral = manipulator.hasCoral();
        this.isArmSafeForElevatorUp = manipulator.arm().gte(armSafeElevatorDegrees);
        this.isArmSafeForWristDown = manipulator.arm().lte(armSafeDownDegrees);
        this.isArmSafeForWristUp = manipulator.arm().gte(armSafeUpDegrees);
        this.isArmSafeForWristDownBack = manipulator.arm().gte(armSafeDownDegreesBack);
        this.isArmSafeForWristUpBack = manipulator.arm().lte(armSafeUpDegreesBack);
    }

    public Command floor() {
        return sequence(
                parallel(elevatorFloor(),
                        armFloor())
                        .until(isArmSafeForWristDown),
                parallel(elevatorFloor(),
                        armFloor(),
                        claws(),
                        manipulator.wrist().toAngle(wristPickupDegrees),
                        intake()))
                .withName("Coral Pickup: Floor");
    }

    public Command station() {
        return sequence(
                parallel(elevatorStation(),
                        armStation())
                        .until(isArmSafeForWristDown),
                parallel(elevatorStation(),
                        armStation(),
                        claws(),
                        manipulator.wrist().toAngle(wristPickupDegrees),
                        intake()))
                .withName("Coral Pickup: Station ");
    }

    public Command stationBack() {
        return sequence(
                parallel(elevatorStationBack(),
                        armStationBack())
                        .until(isArmSafeForWristDownBack),
                parallel(elevatorStationBack(),
                        armStationBack(),
                        claws(),
                        manipulator.wrist().toAngle(wristPickupDegreesBack),
                        intake()))
                .withName("Coral Pickup: Station Back");
    }

    public Command hold() {
        return sequence(
                parallel(armHold(),
                        intakeHold())
                        .until(isArmSafeForWristUp),
                parallel(armHold(),
                        algaeClawHold(),
                        intakeHold(),
                        manipulator.wrist().toAngle(wristSafeDegrees)))
                .withName("Coral Hold");
    }

    public Command holdFromBack() {
        return sequence(
                parallel(armHold(),
                        intakeHold())
                        .until(isArmSafeForWristUpBack),
                parallel(armHold(),
                        algaeClawHold(),
                        intakeHold(),
                        manipulator.wrist().toAngle(wristSafeDegrees)))
                .withName("Coral Hold");
    }

    public Command hardReset() {
        return sequence(
                parallel(armHold(),
                        intakeHold())
                        .until(isArmSafeForElevatorUp),
                parallel(elevatorHold(),
                        armHold(),
                        intakeHold())
                        .until(isArmSafeForWristUp),
                parallel(elevatorHold(),
                        armHold(),
                        algaeClawHold(),
                        intakeHold(),
                        manipulator.wrist().toAngle(wristSafeDegrees)))
                .withName("Coral Hard Reset");
    }

    private Command intake() {
        return manipulator
                        .coralIntakeWheel()
                        .toVelocity(() -> coralSpeed).until(hasCoral);
    }

    private Command intakeHold() {
        return manipulator.coralIntakeWheel().toVelocity(() -> coralHoldSpeed);
    }

    private Command claws() {
        return parallel(
                manipulator.algaeClaw().toValue(algaeClawIntakeValue),
                manipulator.coralClaw().toValue(coralClawValue));
    }

    private Command algaeClawHold() {
        return manipulator.algaeClaw().toValue(algaeClawHoldValue);
    }

    private Command elevatorStation() {
        return manipulator.elevator().toHeight(elevatorStationCM / 100.0);
    }

    private Command elevatorStationBack(){
        return manipulator.elevator().toHeight(elevatorStationBackCM / 100.0);
    }

    private Command elevatorFloor() {
        return manipulator.elevator().toHeight(elevatorFloorCM / 100.0);
    }

    private Command elevatorHold() {
        return manipulator.elevator().toHeight(elevatorHoldCM / 100.0);
    }

    private Command armStation() {
        return manipulator.arm().toAngle(armStationDegrees);
    }

    private Command armStationBack() {
        return manipulator.arm().toAngle(armStationDegreesBack);
    }

    private Command armFloor() {
        return manipulator.arm().toAngle(armFloorDegrees);
    }

    private Command armHold() {
        return manipulator.arm().toAngle(armHoldDegrees);
    }
}
