package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Manipulator;

import static digilib.claws.Claw.*;
import static digilib.claws.Claw.Value.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AlgaePickup {
    private static final double armFloorDegrees = -39;
    private static final double armL2Degrees = -7.0;
    private static final double armL3Degrees = -10.0;
    private static final double armSafeDownDegrees = 60.0;
    private static final double armForHoldAfterPickupDegrees = -9.4;

    private static final double elevatorFloorCM = 18.0;
    private static final double elevatorL2CM = 32.0;
    private static final double elevatorL3CM = 56.0;
    private static final double elevatorHoldCM = 5.0;

    private static final double wristPickupDegrees = 90.0;

    private static final double coralSpeed = -1;

    private static final Value algaeClawValue = CLOSED;
    private static final Value coralClawValue = OPEN;

    private final Manipulator manipulator;
    private final Trigger isArmSafeForWristDown;

    public AlgaePickup(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.isArmSafeForWristDown = manipulator.arm().lte(armSafeDownDegrees);
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
                .withName("Algae Pickup: Floor");
    }

    public Command L2() {
        return sequence(
                parallel(elevatorL2(),
                        armL2())
                        .until(isArmSafeForWristDown),
                parallel(elevatorL2(),
                        armL2(),
                        claws(),
                        manipulator.wrist().toAngle(wristPickupDegrees),
                        intake()))
                .withName("Algae Pickup: L2 ");

    }

    public Command L3() {
        return sequence(
                parallel(elevatorL3(),
                        armL3())
                        .until(isArmSafeForWristDown),
                parallel(elevatorL3(),
                        armL3(),
                        claws(),
                        manipulator.wrist().toAngle(wristPickupDegrees),
                        intake()))
                .withName("Algae Pickup: L3 ");

    }

    public Command hold() {
        return parallel(
                elevatorHold(),
                armHold())
                .withName("Algae Hold");
    }

    private Command intake() {
        return sequence(
                race(
                        parallel(
                                manipulator.coralIntakeWheel().toVelocity(() -> coralSpeed)),
                        waitSeconds(0.5)),
                parallel(
                        manipulator.coralIntakeWheel().toVelocity(() -> coralSpeed)));
    }

    private Command claws() {
        return parallel(
                manipulator.algaeClaw().toValue(algaeClawValue),
                manipulator.coralClaw().toValue(coralClawValue));
    }

    private Command elevatorL2() {
        return manipulator.elevator().toHeight(elevatorL2CM / 100.0);
    }

    private Command elevatorL3() {
        return manipulator.elevator().toHeight(elevatorL3CM / 100.0);
    }

    private Command elevatorHold() {
        return manipulator.elevator().toHeight(elevatorHoldCM / 100.0);
    }

    private Command elevatorFloor() {
        return manipulator.elevator().toHeight(elevatorFloorCM / 100.0);
    }

    private Command armL2() {
        return manipulator.arm().toAngle(armL2Degrees);
    }

    private Command armL3() {
        return manipulator.arm().toAngle(armL3Degrees);
    }

    private Command armHold() {
        return manipulator.arm().toAngle(armForHoldAfterPickupDegrees);
    }

    private Command armFloor() {
        return manipulator.arm().toAngle(armFloorDegrees);
    }
}
