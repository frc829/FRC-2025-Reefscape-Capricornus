package frc.robot.commands.game;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.system.Manipulator;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.either;

public class Manual {

    private final Manipulator manipulator;
    public final Trigger hasAlgae;
    public final Trigger hasCoral;

    public Manual(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.hasAlgae = manipulator.hasAlgae;
        this.hasCoral = manipulator.hasCoral;
    }

    public Command manualArm(Supplier<Dimensionless> maxPercent) {
        return manipulator.armToSpeed(maxPercent);
    }

    public Command manualElevator(Supplier<Dimensionless> maxPercent) {
        return manipulator.elevatorToSpeed(maxPercent);
    }

    public Command manualWristToggle() {
        return either(
                manipulator.wristTo(Degrees.of(90.0)),
                manipulator.wristTo(Degrees.of(0.0)),
                manipulator.wristAtAngle(Degrees.of(0.0), Degrees.of(2.0)))
                .withName("Manual Wrist Toggle");
    }

    public Command manualWrist(Supplier<Dimensionless> maxPercent) {
        return manipulator.wristToSpeed(maxPercent);
    }

    public Command manualIntake(
            Dimensionless maxWheel0Percent,
            Dimensionless maxWheel1Percent) {
        return manipulator.intakeToSpeed(maxWheel0Percent, maxWheel1Percent);
    }

    public Command manualAlgaeClawToggle() {
        return manipulator.toggleAlgaeClaw();
    }

    public Command manualCoralClawToggle() {
        return manipulator.toggleCoralClaw();
    }

    public Command manualElevatorTest(){
        return manipulator.elevatorTo(Centimeters.of(20.0));
    }

    public Command manualArmTest(){
        return manipulator.armTo(Degrees.of(30.0));
    }
}
