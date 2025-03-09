package frc.robot.commands;

import digilib.claws.ClawState.ClawValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Manipulator;

import static digilib.claws.ClawState.ClawValue.CLOSED;
import static digilib.claws.ClawState.ClawValue.OPEN;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class CoralPickup {

    private static final double armFloorDegrees = -39;
    private static final double armStationDegrees = 50.0;
    private static final double armHoldDegrees = 90.0;
    private static final double armSafeDownDegrees = 60.0;
    private static final double armSafeUpDegrees = 0.0;
    private static final double armSafeElevatorDegrees = -20.0;

    private static final double elevatorFloorCM = 17.0;
    private static final double elevatorStationCM = 14.0;
    private static final double elevatorHoldCM = 1.0;

    private static final double wristPickupDegrees = 90.0;
    private static final double wristSafeDegrees = 0.0;

    private static final double coralSpeed = 1;

    private static final double coralHoldSpeed = 0.05;

    private static final ClawValue algaeClawIntakeValue = OPEN;
    private static final ClawValue algaeClawHoldValue = OPEN;
    private static final ClawValue coralClawValue = CLOSED;

    private final Manipulator manipulator;
    public final Trigger hasCoral;
    private final Trigger isArmSafeForElevatorUp;
    private final Trigger isArmSafeForWristDown;
    private final Trigger isArmSafeForWristUp;

    public CoralPickup(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.hasCoral = manipulator.hasCoral();
        this.isArmSafeForElevatorUp = manipulator.arm().gte(armSafeElevatorDegrees);
        this.isArmSafeForWristDown = manipulator.arm().lte(armSafeDownDegrees);
        this.isArmSafeForWristUp = manipulator.arm().gte(armSafeUpDegrees);

        // Trigger hasCoral = manipulator.lidarSensor().inRange(-10, 20);
        // hasCoral.whileTrue()
        //
        // manipulator.lidarSensor().inRange(-10, 20).whileTrue(
        //         Commands.run(() -> SmartDashboard.putBoolean("Has Coral", true)));
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

    public Command hardReset(){
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
        return manipulator.coralIntakeWheel().toVelocity(() -> coralSpeed)
                .until(hasCoral);
    }

    private Command intakeHold() {
        return manipulator.coralIntakeWheel().toVelocity(() -> coralHoldSpeed);
    }

    private Command claws() {
        return parallel(
                manipulator.algaeClaw().toClawValue(algaeClawIntakeValue),
                manipulator.coralClaw().toClawValue(coralClawValue));
    }

    private Command algaeClawHold() {
        return manipulator.algaeClaw().toClawValue(algaeClawHoldValue);
    }

    private Command elevatorStation() {
        return manipulator.elevator().toHeight(elevatorStationCM / 100.0);
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

    private Command armFloor() {
        return manipulator.arm().toAngle(armFloorDegrees);
    }

    private Command armHold() {
        return manipulator.arm().toAngle(armHoldDegrees);
    }
}
