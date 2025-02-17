package frc.robot.commandFactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

public class PickupFactories {
    public final SubsystemCommandFactories subs;

    public PickupFactories(SubsystemCommandFactories subs) {
        this.subs = subs;
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
                        .andThen(subs.wrist.goToAngle(Degrees.of(0.0), Degrees.of(2.0)).asProxy()
                                .alongWith(subs.elevator.goToPosition(Centimeters.of(20), Centimeters.of(2.0)).asProxy())
                                .until(subs.wrist.atPosition(Degrees.of(0.0), Degrees.of(2.0)))
                                .andThen(subs.dualIntake.moveAtVelocity(
                                                () -> 0.0,
                                                () -> 0.25)
                                        .alongWith(subs.arm.goToAngle(Degrees.of(20.0), Degrees.of(2.0)).asProxy(),
                                                subs.elevator.goToPosition(Centimeters.of(20), Centimeters.of(2.0)).asProxy()))
                                .until(subs.dualIntake.hasCoral))
                        .withName("Coral Station Front");

    }

    public Command coralStationBack() {
        return Commands.none();
    }

    private Command closeClaws() {
        return subs.algae.close().alongWith(subs.coral.close());
    }
}
