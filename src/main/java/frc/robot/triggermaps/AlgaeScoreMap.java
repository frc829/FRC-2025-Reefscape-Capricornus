package frc.robot.triggermaps;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgaeScore;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftTrigger;

public class AlgaeScoreMap {

    private final CommandXboxController driver;
    private final CommandXboxController operator;
    private final double deadband;
    private final AlgaeScore algae;

    public AlgaeScoreMap(
            CommandXboxController driver,
            CommandXboxController operator,
            double deadband,
            AlgaeScore algae) {
        this.driver = driver;
        this.operator = operator;
        this.deadband = deadband;
        this.algae = algae;

        bindBargeAlign();
        bindBargeScore();
        bindProcessorScore();
    }

    private void bindBargeAlign() {
        operator.povLeft()
                .whileTrue(algae.bargeAlign());
    }

    private void bindBargeScore() {
        operator.axisMagnitudeGreaterThan(kLeftTrigger.value, deadband)
                .whileTrue(algae.score())
                .onFalse(algae.reset());
    }

    private void bindProcessorScore() {
        driver.rightBumper()
                .whileTrue(algae.score())
                .onFalse(algae.reset());
    }

}
