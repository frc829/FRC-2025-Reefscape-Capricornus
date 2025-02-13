package frc.robot.routines;

import frc.robot.controllers.OperatorFlightStickController;
import frc.robot.controllers.OperatorXboxController;
import frc.robot.commandFactories.ScoringFactories;

public class ScoringRoutines {

    public ScoringRoutines(
            OperatorXboxController operatorController,
            OperatorFlightStickController flightStickController,
            ScoringFactories score) {
        // operatorController.bargeScore()
        //         .whileTrue(score.bargeAlign())
        //         .onFalse(score.bargeScore());
        // operatorController.processorScore()
        //         .whileTrue(score.processorAlign())
        //         .onFalse(score.processorScore());
        // operatorController.l1Score()
        //         .whileTrue(score.l1Align())
        //         .onFalse(score.coralScore());
        // operatorController.l2Score()
        //         .whileTrue(score.l2Align())
        //         .onFalse(score.coralScore());
        // operatorController.l3Score()
        //         .whileTrue(score.l3Align())
        //         .onFalse(score.coralScore());
        // operatorController.l4Score()
        //         .whileTrue(score.l4Align())
        //         .onFalse(score.coralScore());
    }
}
