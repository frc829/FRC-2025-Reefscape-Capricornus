package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

public class PickupFactories {
    public final SubsystemCommandFactories subs;
    private final Trigger wristAtZero;
    private final Trigger wristAt90;

    public PickupFactories(SubsystemCommandFactories subs) {
        this.subs = subs;
        wristAtZero = subs.wrist.atPosition(Degrees.of(0.0), Degrees.of(2.0));
        wristAt90 = subs.wrist.atPosition(Degrees.of(90.0), Degrees.of(2.0));
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
        return Commands.none();

    }

    public Command coralStationFront() {
        return
                closeClaws()
                        .andThen(wristToZero()
                                .alongWith(elevatorTo(Centimeters.of(20.0), Centimeters.of(2.0)))
                                .until(wristAtZero))
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
                                .alongWith(wristTo90())
                                .until(wristAt90))
                        .andThen(subs.dualIntake.moveAtVelocity(
                                        () -> 0.0,
                                        () -> 0.25)
                                .until(subs.dualIntake.hasCoral))
                        .withName("Coral Station Front");

    }

    public Command coralStationBack() {
        return Commands.none();
    }

    private Command closeClaws() {
        return subs.algae.close().alongWith(subs.coral.close());
    }

    private Command wristToZero() {
        return subs.wrist.goToAngle(Degrees.of(0.0), Degrees.of(2.0)).asProxy();
    }

    private Command wristTo90() {
        return subs.wrist.goToAngle(Degrees.of(90.0), Degrees.of(2.0)).asProxy();
    }

    private Command elevatorTo(Distance position, Distance tolerance) {
        return subs.elevator.goToPosition(position, tolerance).asProxy();
    }
}
