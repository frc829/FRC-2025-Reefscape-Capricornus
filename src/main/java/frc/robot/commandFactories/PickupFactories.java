package frc.robot.commandFactories;

import digilib.claws.ClawValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static digilib.claws.ClawValue.*;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class PickupFactories {

    private static final ClawValue algaeClawCoralIntake = CLOSED;
    private static final ClawValue algaeClawAlgaeIntake = CLOSED;
    private static final ClawValue coralClawCoralIntake = CLOSED;
    private static final ClawValue coralClawAlgaeIntake = CLOSED;

    private static final Angle wristSafe = Degrees.of(0.0);
    private static final Angle wristPickup = Degrees.of(90.0);
    private static final Angle wristTolerance = Degrees.of(2.0);

    private static final Distance elevatorCoralFloor = Centimeters.of(20.0);
    private static final Distance elevatorCoralStation = Centimeters.of(0.0);
    private static final Distance elevatorAlgaeFloor = Centimeters.of(0.0);
    private static final Distance elevatorAlgaeL2 = Centimeters.of(0.0);
    private static final Distance elevatorAlgaeL3 = Centimeters.of(0.0);
    private static final Distance elevatorAlgaeHold = Centimeters.of(0.0);
    private static final Distance elevatorCoralHold = Centimeters.of(0.0);
    private static final Distance elevatorTolerance = Centimeters.of(1.0);

    private static final Angle armCoralFloor = Degrees.of(-30.0);
    private static final Angle armCoralStation = Degrees.of(0.0);
    private static final Angle armAlgaeFloor = Degrees.of(0.0);
    private static final Angle armAlgaeL2 = Degrees.of(0.0);
    private static final Angle armAlgaeL3 = Degrees.of(0.0);
    private static final Angle armAlgaeHold = Degrees.of(0.0);
    private static final Angle armCoralHold = Degrees.of(90.0);
    private static final Angle armTolerance = Degrees.of(2.0);

    private static final Dimensionless intakeWheel0AlgaeSpeed = Percent.of(25);
    private static final Dimensionless intakeWheel1AlgaeSpeed = Percent.of(25);
    private static final Dimensionless intakeWheel0CoralSpeed = Percent.of(0);
    private static final Dimensionless intakeWheel1CoralSpeed = Percent.of(25);


    private final ManipulatorFactories manip;
    private final Trigger hasCoral;
    private final Trigger hasAlgae;
    private final Trigger isCoralClawClosed;
    private final Trigger isAlgaeClawClosed;
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
    private final Trigger isArmAtAlgaeStore;
    private final Trigger isArmAtCoralStore;
    private final Trigger isElevatorAtAlgaeStore;
    private final Trigger isElevatorAtCoralStore;


    public PickupFactories(ManipulatorFactories manip) {
        this.manip = manip;
        this.hasAlgae = manip.hasAlgae;
        this.hasCoral = manip.hasCoral;
        this.isCoralClawClosed = manip.isCoralClawClosed;
        this.isAlgaeClawClosed = manip.isAlgaeClawClosed;
        this.isWristSafe = manip.wristAtAngle(wristSafe, wristTolerance);
        this.isWristPickup = manip.wristAtAngle(wristPickup, wristTolerance);
        this.isArmAtCoralStation = manip.armAtAngle(armCoralStation, armTolerance);
        this.isElevatorAtCoralStation = manip.elevatorAtHeight(elevatorCoralStation, elevatorTolerance);
        this.isArmAtCoralFloor = manip.armAtAngle(armCoralFloor, armTolerance);
        this.isElevatorAtCoralFloor = manip.elevatorAtHeight(elevatorCoralFloor, elevatorTolerance);
        this.isArmAtAlgaeFloor = manip.armAtAngle(armAlgaeFloor, armTolerance);
        this.isElevatorAtAlgaeFloor = manip.elevatorAtHeight(elevatorAlgaeFloor, elevatorTolerance);
        this.isArmAtAlgaeL2 = manip.armAtAngle(armAlgaeL2, armTolerance);
        this.isElevatorAtAlgaeL2 = manip.elevatorAtHeight(elevatorAlgaeL2, elevatorTolerance);
        this.isArmAtAlgaeL3 = manip.armAtAngle(armAlgaeL3, armTolerance);
        this.isElevatorAtAlgaeL3 = manip.elevatorAtHeight(elevatorAlgaeL3, elevatorTolerance);
        this.isArmAtAlgaeStore = manip.armAtAngle(armAlgaeHold, armTolerance);
        this.isArmAtCoralStore = manip.armAtAngle(armCoralHold, armTolerance);
        this.isElevatorAtAlgaeStore = manip.elevatorAtHeight(elevatorAlgaeHold, elevatorTolerance);
        this.isElevatorAtCoralStore = manip.elevatorAtHeight(elevatorCoralHold, elevatorTolerance);
    }

    public Command algaeFloor() {
        return sequence(
                clawsForAlgae(),
                parallel(elevatorAlgaeFloor(), wristSafe()).until(isWristSafe),
                parallel(elevatorAlgaeFloor(), armAlgaeFloor()).until(isElevatorAtAlgaeFloor.and(isArmAtAlgaeFloor).and(isWristSafe)),
                wristPickup(),
                intakeAlgae().until(hasAlgae))
                .withName("Pickup: Algae Floor");
    }

    public Command algaeL2() {
        return sequence(
                clawsForAlgae(),
                parallel(elevatorAlgaeL2(), wristSafe()).until(isWristSafe),
                parallel(elevatorAlgaeL2(), armAlgaeL2()).until(isElevatorAtAlgaeL2.and(isArmAtAlgaeL2).and(isWristSafe)),
                wristPickup(),
                intakeAlgae().until(hasAlgae))
                .withName("Pickup: Algae L2");
    }

    public Command algaeL3() {
        return sequence(
                clawsForAlgae(),
                parallel(elevatorAlgaeL3(), wristSafe()).until(isWristSafe),
                parallel(elevatorAlgaeL3(), armAlgaeL3()).until(isElevatorAtAlgaeL3.and(isArmAtAlgaeL3).and(isWristSafe)),
                wristPickup(),
                intakeAlgae().until(hasAlgae))
                .withName("Pickup: Algae L3");
    }

    public Command coralFloor() {
        return either(
                sequence(
                        intakeCoral().until(hasCoral)),
                sequence(
                        clawsForCoral(),
                        parallel(elevatorCoralFloor(), wristSafe()).until(isWristSafe),
                        parallel(elevatorCoralFloor(), armCoralFloor()).until(isElevatorAtCoralFloor.and(isArmAtCoralFloor).and(isWristSafe)),
                        wristPickup(),
                        intakeCoral().until(hasCoral)),
                isWristPickup.and(isArmAtCoralFloor).and(isElevatorAtCoralFloor).and(isAlgaeClawClosed).and(isCoralClawClosed))
                .withName("Pickup: Coral Floor");
    }

    public Command coralStore() {
        return sequence(
                arm45Degrees(),
                wristSafe(),
                parallel(elevatorCoralHold(), armCoralHold()))
                .withName("Store: Coral Floor");
    }


    public Command coralStation() {
        return sequence(
                clawsForCoral(),
                parallel(elevatorCoralStation(), wristSafe()).until(isWristSafe),
                parallel(elevatorCoralStation(), armCoralStation()).until(isElevatorAtCoralStation.and(isArmAtCoralStation).and(isWristSafe)),
                wristPickup(),
                intakeCoral().until(hasCoral))
                .withName("Pickup: Coral Station");
    }


    public Command holdAfterAlgae() {
        return Commands.none();
    }

    private Command clawsForAlgae() {
        return parallel(manip.setAlgaeClaw(algaeClawAlgaeIntake), manip.setCoralClaw(coralClawAlgaeIntake));
    }

    private Command clawsForCoral() {
        return parallel(manip.setAlgaeClaw(algaeClawCoralIntake), manip.setCoralClaw(coralClawCoralIntake));
    }

    private Command elevatorCoralHold() {
        return manip.elevatorTo(elevatorCoralHold, elevatorTolerance);
    }

    private Command armCoralHold() {
        return manip.armTo(armCoralHold, armTolerance);
    }

    private Command elevatorCoralFloor() {
        return manip.elevatorTo(elevatorCoralFloor, elevatorTolerance);
    }

    private Command elevatorAlgaeFloor() {
        return manip.elevatorTo(elevatorAlgaeFloor, elevatorTolerance);
    }

    private Command elevatorCoralStation() {
        return manip.elevatorTo(elevatorCoralStation, elevatorTolerance);
    }

    private Command elevatorAlgaeL2() {
        return manip.elevatorTo(elevatorAlgaeL2, elevatorTolerance);
    }

    private Command elevatorAlgaeL3() {
        return manip.elevatorTo(elevatorAlgaeL3, elevatorTolerance);
    }

    private Command armCoralFloor() {
        return manip.armTo(armCoralFloor, armTolerance);
    }

    private Command armAlgaeFloor() {
        return manip.armTo(armCoralFloor, armTolerance);
    }

    private Command armCoralStation() {
        return manip.armTo(armCoralStation, armTolerance);
    }

    private Command armAlgaeL2() {
        return manip.armTo(armAlgaeL2, armTolerance);
    }

    private Command armAlgaeL3() {
        return manip.armTo(armAlgaeL3, armTolerance);
    }

    private Command arm45Degrees() {
        return manip.armTo(Degrees.of(45), Degrees.of(2));
    }

    private Command wristPickup() {
        return manip.wristTo(wristPickup, wristTolerance);
    }

    private Command wristSafe() {
        return manip.wristTo(wristSafe, wristTolerance);
    }

    private Command intakeAlgae() {
        return manip.intakeToSpeed(intakeWheel0AlgaeSpeed, intakeWheel1AlgaeSpeed);
    }

    private Command intakeCoral() {
        return manip.intakeToSpeed(intakeWheel0CoralSpeed, intakeWheel1CoralSpeed);
    }


}
