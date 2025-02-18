package frc.robot.commandFactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class PickupFactories {

    private static final double wristSafeDeg = 0.0;
    private static final double wristPickupDeg = 90.0;
    private static final double wristToleranceDegrees = 2.0;

    private static final double elevatorCoralFloorCentimeters = 0.0;
    private static final double elevatorCoralStationCentimeters = 0.0;
    private static final double elevatorAlgaeFloorCentimeters = 0.0;
    private static final double elevatorAlgaeL2Centimeters = 0.0;
    private static final double elevatorAlgaeL3Centimeters = 0.0;
    private static final double elevatorToleranceCentimeters = 1.0;

    private static final double armCoralFloorDegrees = 0.0;
    private static final double armCoralStationDegrees = 0.0;
    private static final double armAlgaeFloorDegrees = 0.0;
    private static final double armAlgaeL2Degrees = 0.0;
    private static final double armAlgaeL3Degrees = 0.0;
    private static final double armToleranceDegrees = 2.0;


    private final SubsystemCommandFactories subs;
    private final Trigger hasCoral;
    private final Trigger hasAlgae;
    private final Trigger isWristSafe;
    private final Trigger isWristPickup;
    private final Trigger isArmAtCoralStation;
    private final Trigger isElevatorAtCoralStation;
    private final Trigger isArmAtCoralFloor;
    private final Trigger isElevatorAtCoralFloor;
    private final Trigger isArmAtAlgaeFloor;
    private final Trigger isElevatorAtAlgaeFloor;
    private final Trigger isArmAtAlgaeL2;
    private final Trigger isElevatorAtAlgaeL2;
    private final Trigger isArmAtAlgaeL3;
    private final Trigger isElevatorAtAlgaeL3;


    public PickupFactories(SubsystemCommandFactories subs) {
        this.subs = subs;
        this.hasAlgae = subs.hasAlgae;
        this.hasCoral = subs.hasCoral;
        this.isWristSafe = subs.wristAtAngle(wristSafeDeg, wristToleranceDegrees);
        this.isWristPickup = subs.wristAtAngle(wristPickupDeg, wristToleranceDegrees);
        this.isArmAtCoralStation = subs.armAtAngle(armCoralStationDegrees, armToleranceDegrees);
        this.isElevatorAtCoralStation = subs.elevatorAtHeight(elevatorCoralStationCentimeters, elevatorToleranceCentimeters);
        this.isArmAtCoralFloor = subs.armAtAngle(armCoralFloorDegrees, armToleranceDegrees);
        this.isElevatorAtCoralFloor= subs.elevatorAtHeight(ar, elevatorToleranceCentimeters);
        this.isArmAtAlgaeFloor = subs.armAtAngle(0.0, armToleranceDegrees);
        this.isElevatorAtAlgaeFloor = subs.elevatorAtHeight(0.0, elevatorToleranceCentimeters);
        this.isArmAtAlgaeL2 = subs.armAtAngle(0.0, armToleranceDegrees);
        this.isElevatorAtAlgaeL2 = subs.elevatorAtHeight(0.0, elevatorToleranceCentimeters);
        this.isArmAtAlgaeL3 = subs.armAtAngle(0.0, armToleranceDegrees);
        this.isElevatorAtAlgaeL3 = subs.elevatorAtHeight(0.0, elevatorToleranceCentimeters);
    }

    public Command algaeFloor() {
        return Commands.none();

    }

    public Command algaeL2() {
        return Commands.none();

    }

    public Command algaeL3() {
        return Commands.none();

    }

    public Command coralFloor() {
        return sequence(
                clawsForCoralPickup(),
                parallel(elevatorCoralFloor(), wristSafe()).until(isWristSafe),
                parallel(elevatorCoralFloor(), armCoralFloor()).until()
        ).withName("Pickup: Coral Floor");

    }

    public Command coralStation() {
        return
                closeClaws()
                        .andThen(subs.wristToZero()
                                .alongWith(elevatorTo(Centimeters.of(20.0), Centimeters.of(2.0)))
                                .until(subs.wristAt0Deg))
                        .andThen(subs.dualIntake.moveAtVelocity(
                                        () -> 0.0,
                                        () -> 0.25)
                                .alongWith(subs.arm.goToAngle(Degrees.of(20.0), Degrees.of(2.0)).asProxy(),
                                        subs.elevator.goToPosition(Centimeters.of(20), Centimeters.of(2.0)).asProxy())
                                .until(subs.elevator.atPosition(Centimeters.of(20.0), Centimeters.of(2.0))
                                        .and(subs.arm.atPosition(Degrees.of(20.0), Degrees.of(2.0)))))
                        .andThen(subs.dualIntake.moveAtVelocity(
                                        () -> 0.0,
                                        () -> 0.25)
                                .alongWith(subs.wristTo90())
                                .until(isWristPickup))
                        .andThen(subs.dualIntake.moveAtVelocity(
                                        () -> 0.0,
                                        () -> 0.25)
                                .until(subs.dualIntake.hasCoral))
                        .withName("Coral Station Front");

    }

    private Command clawsForCoralPickup() {
        return Commands.none();
    }

    private Command clawsForAlgaePickup() {
        return Commands.none();
    }

    private Command elevatorCoralFloor() {
        return Commands.none();
    }

    private Command elevatorAlgaeFloor() {
        return Commands.none();
    }

    private Command elevatorCoralStation() {
        return Commands.none();
    }

    private Command elevatorAlgaeL2() {
        return Commands.none();
    }

    private Command elevatorAlgaeL3() {
        return Commands.none();
    }

    private Command armCoralFloor() {
        return Commands.none();
    }

    private Command armAlgaeFloor() {
        return Commands.none();
    }

    private Command armCoralStation() {
        return Commands.none();
    }

    private Command armAlgaeL2() {
        return Commands.none();
    }

    private Command armAlgaeL3() {
        return Commands.none();
    }

    private Command wristPickup() {
        return Commands.none();
    }

    private Command wristSafe() {
        return Commands.none();
    }

    private Command intakeAlgae() {
        return Commands.none();
    }

    private Command intakeCoral() {
        return Commands.none();
    }
}
