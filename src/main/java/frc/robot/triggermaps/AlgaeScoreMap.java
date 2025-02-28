package frc.robot.triggermaps;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.game.AlgaeScore;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftTrigger;

public class AlgaeScoreMap {

    private final CommandXboxController driver;
    private final CommandXboxController operator;
    private final double deadband;
    private final AlgaeScore algae;
    private final Supplier<Command> coralHold;

    public AlgaeScoreMap(
            CommandXboxController driver,
            CommandXboxController operator,
            double deadband,
            AlgaeScore algae,
            Supplier<Command> coralHold) {
        this.driver = driver;
        this.operator = operator;
        this.deadband = deadband;
        this.algae = algae;
        this.coralHold = coralHold;


        bindBargeAlign();
        bindBargeScore();
        bindProcessorAlign();
        bindProcessorScore();
    }

    private void bindBargeAlign() {
        operator.povLeft()
                .whileTrue(algae.bargeAlign());
    }

    private void bindBargeScore() {
        operator.axisMagnitudeGreaterThan(kLeftTrigger.value, deadband)
                .whileTrue(algae.bargeScore())
                .onFalse(algae.bargeScoreReset());
    }

    private void bindProcessorAlign() {
        operator.povDown().whileTrue(algae.processorAlign())
                .onFalse(coralHold.get());
    }


    private void bindProcessorScore() {
        driver.rightBumper().whileTrue(algae.processorScore());
    }

}
