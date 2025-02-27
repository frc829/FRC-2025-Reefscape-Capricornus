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

    private static final ClawValue algaeClawCoralIntake = OPEN;
    private static final ClawValue coralClawCoralIntake = CLOSED;

    private static final Angle wristSafe = Degrees.of(0.0);
    private static final Angle wristPickup = Degrees.of(90.0);
    private static final Angle wristTolerance = Degrees.of(4.0);

    private static final Distance elevatorCoralFloor = Centimeters.of(15.0);
    private static final Distance elevatorCoralStation = Centimeters.of(5.0);
    private static final Distance elevatorCoralHold = Centimeters.of(1.0);

    private static final Angle armCoralFloor = Degrees.of(-43.7);
    private static final Angle armCoralStation = Degrees.of(50.0);
    private static final Angle armCoralHold = Degrees.of(90.0);


    private static final Dimensionless intakeWheel0CoralSpeed = Percent.of(0);
    private static final Dimensionless intakeWheel1CoralSpeed = Percent.of(75);

    private final Manipulator manipulator;

    private final Trigger armSafeForWristDown;
    private final Trigger armSafeForWristUp;

    public final Trigger hasCoral;
    private final Trigger isCoralClawClosed;
    private final Trigger isAlgaeClawClosed;
    private final Trigger isWristSafe;
    private final Trigger isWristPickup;
    private final Trigger isArmAtCoralStation;
    private final Trigger isElevatorAtCoralStation;
    private final Trigger isArmAtCoralFloor;
    private final Trigger isElevatorAtCoralFloor;
    private final Trigger isArmAtCoralStore;
    private final Trigger isElevatorAtCoralStore;


    public CoralPickup(Manipulator manipulator) {
        this.manipulator = manipulator;

        this.armSafeForWristDown = manipulator.armLessThan(Degrees.of(60.0));
        this.armSafeForWristUp = manipulator.armGreaterThan(Degrees.of(10.0));

        this.hasCoral = manipulator.hasCoral;
        this.isCoralClawClosed = manipulator.isCoralClawClosed;
        this.isAlgaeClawClosed = manipulator.isAlgaeClawClosed;
        this.isWristSafe = manipulator.wristAtAngle(wristSafe, wristTolerance);
        this.isWristPickup = manipulator.wristAtAngle(wristPickup, wristTolerance);
        this.isArmAtCoralStation = manipulator.armAtAngle(armCoralStation);
        this.isElevatorAtCoralStation = manipulator.elevatorAtHeight(elevatorCoralStation);
        this.isArmAtCoralFloor = manipulator.armAtAngle(armCoralFloor);
        this.isElevatorAtCoralFloor = manipulator.elevatorAtHeight(elevatorCoralFloor);
        this.isArmAtCoralStore = manipulator.armAtAngle(armCoralHold);
        this.isElevatorAtCoralStore = manipulator.elevatorAtHeight(elevatorCoralHold);
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

    public Command coralHold() {
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





    private Command clawsForCoral() {
        return parallel(manip.setAlgaeClaw(algaeClawCoralIntake), manip.setCoralClaw(coralClawCoralIntake));
    }

    private Command clawsForCoralStore() {
        return parallel(manip.setAlgaeClaw(CLOSED), manip.setCoralClaw(CLOSED));
    }

    private Command elevatorCoralHold() {
        return manip.elevatorTo(elevatorCoralHold);
    }

    private Command armCoralHold() {
        return manip.armTo(armCoralHold);
    }



    private Command elevatorCoralFloor() {
        return manip.elevatorTo(elevatorCoralFloor);
    }


    private Command elevatorCoralStation() {
        return manip.elevatorTo(elevatorCoralStation);
    }



    private Command wristPickup() {
        return manip.wristTo(wristPickup, wristTolerance);
    }

    private Command wristSafe() {
        return manip.wristTo(wristSafe, wristTolerance);
    }


    private Command intakeCoral() {
        return manip.intakeToSpeed(intakeWheel0CoralSpeed, intakeWheel1CoralSpeed);
    }


}
