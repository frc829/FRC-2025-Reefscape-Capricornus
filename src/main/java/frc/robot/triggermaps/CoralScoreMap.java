package frc.robot.triggermaps;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.game.CoralScore;

public class CoralScoreMap {

    private final CommandXboxController driver;
    private final CommandXboxController operator;
    private final CoralScore coral;

    public CoralScoreMap(
            CommandXboxController driver,
            CommandXboxController operator,
            CoralScore coral) {
        this.driver = driver;
        this.operator = operator;
        this.coral = coral;

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
                .onFalse(coral.reset());
    }

    private void bindL2Align() {
        operator.x()
                .whileTrue(coral.l2Align())
                .onFalse(coral.reset());
    }

    private void bindL3Score() {
        operator.b()
                .whileTrue(coral.l3Align())
                .onFalse(coral.reset());
    }

    private void bindL4Score() {
        operator.y()
                .whileTrue(coral.l4Align())
                .onFalse(coral.reset());
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
