package frc.robot.commandFactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class ScoringFactories {

    private final SubsystemCommandFactories factories;
    private final ResetFactories reset;
    private final Angle safeArmAngleForWrist = Degrees.of(10.0);

    public ScoringFactories(SubsystemCommandFactories factories,
                            ResetFactories reset) {
        this.factories = factories;
        this.reset = reset;
    }

    public Command l1Align() {
        return Commands.none();
    }

    public Command l2Align() {
        return Commands.none();
    }

    public Command l3Align() {
        return Commands.none();
    }

    public Command l4Align() {
        return Commands.none();
    }

    public Command coralScore() {
        return Commands.none();
    }

    public Command bargeAlign() {
        return Commands.none();
    }

    public Command processorAlign() {
        return Commands.none();
    }

    public Command bargeScore() {
        return Commands.none();
    }

    public Command processorScore() {
        return Commands.none();
    }

    private Command createCoralAlignSafe(
            Distance elevatorHeight,
            Angle armAngle,
            Angle wristAngle) {
        return Commands.none();

    }

    private Command createCoralAlignUnSafe(
            Distance elevatorHeight,
            Angle armAngle,
            Angle wristAngle) {
        return Commands.none();

    }
}
