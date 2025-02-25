package frc.robot.commandFactories;

import digilib.claws.ClawValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static digilib.claws.ClawValue.*;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class PickupFactories {

    private static final ClawValue algaeClawCoralIntake = OPEN;
    private static final ClawValue algaeClawAlgaeIntake = CLOSED;
    private static final ClawValue coralClawCoralIntake = CLOSED;
    private static final ClawValue coralClawAlgaeIntake = OPEN;

    private static final Angle wristSafe = Degrees.of(0.0);
    private static final Angle wristPickup = Degrees.of(90.0);
    private static final Angle wristTolerance = Degrees.of(4.0);

    private static final Distance elevatorCoralFloor = Centimeters.of(15.0);
    private static final Distance elevatorCoralStation = Centimeters.of(5.0);
    private static final Distance elevatorAlgaeFloor = Centimeters.of(15.0);
    private static final Distance elevatorAlgaeL2 = Centimeters.of(27.0);
    private static final Distance elevatorAlgaeL3 = Centimeters.of(46.0);
    private static final Distance elevatorAlgaeHold = Centimeters.of(5.0);
    private static final Distance elevatorCoralHold = Centimeters.of(1.0);
    private static final Distance elevatorTolerance = Centimeters.of(2.0);


    private static final Angle armCoralFloor = Degrees.of(-43.7);
    private static final Angle armCoralStation = Degrees.of(50.0);
    private static final Angle armAlgaeFloor = Degrees.of(-43.7);
    private static final Angle armAlgaeL2 = Degrees.of(-7.0);
    private static final Angle armAlgaeL3 = Degrees.of(-7.0);
    private static final Angle armAlgaeHold = Degrees.of(0.0);
    private static final Angle armCoralHold = Degrees.of(90.0);
    private static final Angle armTolerance = Degrees.of(3.0);

    private static final Dimensionless intakeWheel0AlgaeSpeed = Percent.of(-100);
    private static final Dimensionless intakeWheel1AlgaeSpeed = Percent.of(-100);
    private static final Dimensionless intakeWheel0CoralSpeed = Percent.of(0);
    private static final Dimensionless intakeWheel1CoralSpeed = Percent.of(75);


    private final ManipulatorFactories manip;

    private final Trigger armSafeForWristDown;
    private final Trigger armSafeForWristUp;

    public final Trigger hasCoral;
    public final Trigger hasAlgae;
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

        this.armSafeForWristDown = manip.armLessThan(Degrees.of(60.0));
        this.armSafeForWristUp = manip.armGreaterThan(Degrees.of(10.0));

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

    public Command algaeIntake(){
        return sequence(
                race(intakeAlgae(), waitSeconds(0.5)),
                intakeAlgae().until(hasAlgae)
        );
    }

    public Command algaeFloor() {
        return sequence(
                parallel(elevatorAlgaeFloor(), armAlgaeFloor(), intakeAlgae())
                        .until(armSafeForWristDown),
                clawsForAlgae(),
                parallel(elevatorAlgaeFloor(), armAlgaeFloor(), algaeIntake(), wristPickup())
                        .until(isElevatorAtAlgaeFloor.and(isArmAtAlgaeFloor).and(isWristPickup)),
                algaeIntake())
                .withName("Pickup: Algae Floor");
    }

    public Command algaeL2() {
        return sequence(
                parallel(elevatorAlgaeL2(), armAlgaeL2(), intakeAlgae())
                        .until(armSafeForWristDown),
                clawsForAlgae(),
                parallel(elevatorAlgaeL2(), armAlgaeL2(), intakeAlgae().until(hasAlgae), wristPickup())
                        .until(isElevatorAtAlgaeL2.and(isArmAtAlgaeL2).and(isWristPickup)),
                intakeAlgae().until(hasAlgae))
                .withName("Pickup: Algae L2");
    }

    public Command algaeL3() {
        return sequence(
                parallel(elevatorAlgaeL3(), armAlgaeL3(), intakeAlgae())
                        .until(armSafeForWristDown),
                clawsForAlgae(),
                parallel(elevatorAlgaeL3(), armAlgaeL3(), intakeAlgae().until(hasAlgae), wristPickup())
                        .until(isElevatorAtAlgaeL3.and(isArmAtAlgaeL3).and(isWristPickup)),
                intakeAlgae().until(hasAlgae))
                .withName("Pickup: Algae L3");
    }

    public Command coralFloor() {
        return sequence(
                clawsForCoral(),
                parallel(elevatorCoralFloor(), armCoralFloor(), intakeCoral())
                        .until(armSafeForWristDown),
                parallel(elevatorCoralFloor(), armCoralFloor(), intakeCoral().until(hasCoral), wristPickup())
                        .until(isElevatorAtCoralFloor.and(isArmAtCoralFloor).and(isWristPickup)),
                intakeCoral().until(hasCoral)
        ).withName("Pickup: Coral Floor");
    }

    public Command coralStore() {
        return sequence(
                clawsForCoralStore(),
                parallel(elevatorCoralHold(), armCoralHold())
                        .until(armSafeForWristUp),
                parallel(elevatorCoralHold(), armCoralHold(), wristSafe())
        ).withName("Pickup: Coral Store");
    }

    public Command coralStation() {
        return sequence(
                clawsForCoral(),
                parallel(elevatorCoralStation(), armCoralStation(), intakeCoral())
                        .until(armSafeForWristDown),
                parallel(elevatorCoralStation(), armCoralStation(), intakeCoral().until(hasCoral), wristPickup())
                        .until(isElevatorAtCoralStation.and(isArmAtCoralStation).and(isWristPickup)),
                intakeCoral().until(hasCoral)
        ).withName("Pickup: Coral Station");
    }

    public Command holdAfterAlgae() {
        return sequence(
                parallel(elevatorAlgaeHold(), armAlgaeHold()))
                .withName("Store: Coral Floor");
    }

    private Command clawsForAlgae() {
        return parallel(manip.setAlgaeClaw(algaeClawAlgaeIntake), manip.setCoralClaw(coralClawAlgaeIntake));
    }

    private Command clawsForCoral() {
        return parallel(manip.setAlgaeClaw(algaeClawCoralIntake), manip.setCoralClaw(coralClawCoralIntake));
    }

    private Command clawsForCoralStore() {
        return parallel(manip.setAlgaeClaw(CLOSED), manip.setCoralClaw(CLOSED));
    }

    private Command elevatorCoralHold() {
        return manip.elevatorTo(elevatorCoralHold, elevatorTolerance);
    }

    private Command armCoralHold() {
        return manip.armTo(armCoralHold, armTolerance);
    }

    private Command elevatorAlgaeHold() {
        return manip.elevatorTo(elevatorAlgaeHold, elevatorTolerance);
    }

    private Command armAlgaeHold() {
        return manip.armTo(armAlgaeHold, armTolerance);
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

    private Command arm0Degrees() {
        return manip.armTo(Degrees.of(0.0), Degrees.of(2));
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
