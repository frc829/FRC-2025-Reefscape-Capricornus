package frc.robot.commandFactories;

import digilib.claws.ClawValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

import static digilib.claws.ClawValue.CLOSED;
import static digilib.claws.ClawValue.OPEN;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ScoringFactories {

    private static final ClawValue coralClawScore = OPEN;

    private static final Angle wristSafe = Degrees.of(0.0);
    private static final Angle wristPickup = Degrees.of(90.0);
    private static final Angle wristTolerance = Degrees.of(4.0);

    private static final Distance elevatorL1 = Centimeters.of(5.0);
    private static final Distance elevatorL2 = Centimeters.of(9.0);
    private static final Distance elevatorL3 = Centimeters.of(30.0);
    private static final Distance elevatorL4 = Centimeters.of(57.0);

    private static final Angle armL1 = Degrees.of(24.0);
    private static final Angle armL2 = Degrees.of(42.6);
    private static final Angle armL3 = Degrees.of(42.6);
    private static final Angle armL4 = Degrees.of(48.0);

    private static final Dimensionless intakeWheel0L1Speed = Percent.of(0.0);
    private static final Dimensionless intakeWheel1L1Speed = Percent.of(-5.0);

    private final ManipulatorFactories factories;
    private final Trigger armAtL1;
    private final Trigger elevatorAtL1;
    private final Trigger elevatorAtL2;
    private final Trigger armAtL2;

    public ScoringFactories(ManipulatorFactories factories) {
        this.factories = factories;
        this.armAtL1 = factories.armAtAngle(armL1, Degrees.of(2.0));
        this.elevatorAtL1 = factories.elevatorAtHeight(elevatorL1, Centimeters.of(1.0));
        this.armAtL2 = factories.armAtAngle(armL2, Degrees.of(2.0));
        this.elevatorAtL2 = factories.elevatorAtHeight(elevatorL2, Centimeters.of(1.0));
    }

    public Command l1Align() {
        return sequence(
                parallel(factories.elevatorTo(elevatorL1, Centimeters.of(1.0)), factories.armTo(armL1, Degrees.of(2.0)))
                        .until(elevatorAtL1.and(armAtL1)),
                wristPickup());
    }

    public Command l1Score() {
        return factories.intakeToSpeed(Percent.of(0.0), Percent.of(-50.0));
    }

    public Command l2Align() {
        return parallel(factories.elevatorTo(elevatorL2, Centimeters.of(1.0)), factories.armTo(armL2, Degrees.of(2.0)))
                .until(elevatorAtL2.and(armAtL2));
    }

    public Command l2Score() {
        return factories.setCoralClaw(OPEN);
    }

    public Command L2ScoreReset(){
        return factories.setCoralClaw(CLOSED);
    }

    public Command l3Align() {
        return none();
    }

    public Command l4Align() {
        return none();
    }

    public Command coralScore() {
        return none();
    }

    public Command bargeAlign() {
        return none();
    }

    public Command processorAlign() {
        return none();
    }

    public Command bargeScore() {
        return none();
    }

    public Command processorScore() {
        return none();
    }

    public Command climb(Supplier<Dimensionless> dutyCycle) {
        return factories.climbAtDutyCycle(dutyCycle);
    }

    private Command wristPickup() {
        return factories.wristTo(wristPickup, wristTolerance);
    }

}
