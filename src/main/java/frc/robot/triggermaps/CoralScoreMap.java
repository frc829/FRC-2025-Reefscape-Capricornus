package frc.robot.triggermaps;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.game.CoralScore;

import java.util.function.Supplier;

public class CoralScoreMap {

    private final CommandXboxController driver;
    private final CommandXboxController operator;
    private final CoralScore coral;
    private final Supplier<Command> coralHold;

    public CoralScoreMap(
            CommandXboxController driver,
            CommandXboxController operator,
            CoralScore coral,
            Supplier<Command> coralHold) {
        this.driver = driver;
        this.operator = operator;
        this.coral = coral;
        this.coralHold = coralHold;

        bindL1Align();
        bindL2Align();
        bindL3Score();
        bindL4Score();
        bindL1Score();
        bindL234Score();
    }

    private void bindL1Align() {
        operator.a()
                .whileTrue(coral.l1Align())
                .onFalse(coralHold.get());
    }

    private void bindL2Align() {
        operator.x()
                .whileTrue(coral.l2Align())
                .onFalse(coralHold.get());
    }

    private void bindL3Score() {
        operator.b()
                .whileTrue(coral.l3Align())
                .onFalse(coralHold.get());
    }

    private void bindL4Score() {
        operator.y()
                .whileTrue(coral.l4Align())
                .onFalse(coralHold.get());
    }

    private void bindL1Score() {
        driver.leftBumper().and(operator.a())
                .whileTrue(coral.l1Score());
    }

    private void bindL234Score() {
        driver.leftBumper().and(operator.a().negate())
                .whileTrue(coral.l234Score());
    }
}
