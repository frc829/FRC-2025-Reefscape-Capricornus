package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.either;

public class ManualFactories {

    private final ManipulatorFactories manip;
    public final Trigger hasAlgae;
    public final Trigger hasCoral;

    public ManualFactories(ManipulatorFactories manip) {
        this.manip = manip;
        this.hasAlgae = manip.hasAlgae;
        this.hasCoral = manip.hasCoral;
    }

    public Command manualArm(Supplier<Dimensionless> maxPercent) {
        return manip.armToSpeed(maxPercent);
    }

    public Command manualElevator(Supplier<Dimensionless> maxPercent) {
        return manip.elevatorToSpeed(maxPercent);
    }

    public Command manualWristToggle() {
        return either(
                manip.wristTo(Degrees.of(90.0), Degrees.of(2.0)),
                manip.wristTo(Degrees.of(0.0), Degrees.of(2.0)),
                manip.wristAtAngle(Degrees.of(0.0), Degrees.of(2.0)))
                .withName("Manual Wrist Toggle");
    }

    public Command manualWrist(Supplier<Dimensionless> maxPercent) {
        return manip.wristToSpeed(maxPercent);
    }

    public Command manualIntake(
            Dimensionless maxWheel0Percent,
            Dimensionless maxWheel1Percent) {
        return manip.intakeToSpeed(maxWheel0Percent, maxWheel1Percent);
    }

    public Command manualAlgaeClawToggle() {
        return manip.toggleAlgaeClaw();
    }

    public Command manualCoralClawToggle() {
        return manip.toggleCoralClaw();
    }

    public Command manualElevatorTest(){
        return manip.elevatorTo(Centimeters.of(20.0), Centimeters.of(0.0));
    }

    public Command manualArmTest(){
        return manip.armTo(Degrees.of(45.0), Degrees.of(0.0));
    }
}
