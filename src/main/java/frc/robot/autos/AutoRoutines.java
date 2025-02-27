package frc.robot.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.game.CoralPickup;
import frc.robot.commands.game.CoralScore;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutoRoutines {

    public enum Points {
        S1,
        S2,
        S3,
        EF,
        GH,
    }

    private final AutoFactory factory;
    private final CoralPickup pickup;
    private final CoralScore score;

    public AutoRoutines(
            AutoFactory factory,
            CoralPickup pickup,
            CoralScore score) {
        this.pickup = pickup;
        this.score = score;

        AutoChooser autoChooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        this.factory = factory;
        autoChooser.addRoutine("The Noob Spot", this::theNoobSpot);
        autoChooser.addRoutine("Noob S1", this::noobS1);
        autoChooser.addRoutine("Noob S3", this::noobS3);
        autoChooser.addRoutine("Plop and Shop", this::plopAndShop);
    }

    private AutoRoutine theNoobSpot() {
        AutoRoutine routine = factory.newRoutine("The Noob Spot");
        AutoTrajectory S2toG = routine.trajectory("S2-to-G");
        Command routineCommand = sequence(S2toG.resetOdometry(), S2toG.cmd());
        routine.active().onTrue(routineCommand);
        S2toG.atTime("Align").onTrue(score.l1Align());
        S2toG.done().onTrue(scoreL1());
        return routine;
    }

    private AutoRoutine noobS1() {
        AutoRoutine routine = factory.newRoutine("Noob S1");
        AutoTrajectory S2toEF = routine.trajectory("S1-to-EF");
        Command routineCommand = sequence(S2toEF.resetOdometry(), S2toEF.cmd());
        routine.active().onTrue(routineCommand);
        S2toEF.atTime("Align").onTrue(score.l1Align());
        S2toEF.done().onTrue(scoreL1());
        return routine;
    }

    private AutoRoutine noobS3() {
        AutoRoutine routine = factory.newRoutine("Noob S3");
        AutoTrajectory S3toIJ = routine.trajectory("S3-to-IJ");
        Command routineCommand = sequence(S3toIJ.resetOdometry(), S3toIJ.cmd());
        routine.active().onTrue(routineCommand);
        S3toIJ.atTime("Align").onTrue(score.l1Align());
        S3toIJ.done().onTrue(scoreL1());
        return routine;
    }

    private AutoRoutine plopAndShop() {
        AutoRoutine routine = factory.newRoutine("Plop and Shop");
        AutoTrajectory S3toIJ = routine.trajectory("S3-to-IJ");
        AutoTrajectory IJtoStationTop = routine.trajectory("IJ-to-Station_Top");
        AutoTrajectory StationToptoLM = routine.trajectory("Station_Top-to-LM");

        Command routineCommand = sequence(
                S3toIJ.resetOdometry(),
                S3toIJ.cmd());

        routine.active().onTrue(routineCommand);
        S3toIJ.atTime("Align").onTrue(score.l1Align());
        S3toIJ.done().onTrue(scoreL1().andThen(IJtoStationTop.cmd()));

        IJtoStationTop.atTime("Pickup").onTrue(pickup.coralStation());
        IJtoStationTop.done().onTrue(waitUntil(pickup.hasCoral).andThen(StationToptoLM.cmd()));

        StationToptoLM.atTime("Align").onTrue(score.l1Align());
        StationToptoLM.done().onTrue(scoreL1());
        return routine;
    }

    private Command scoreL1() {
        return sequence(
                score.l1Score().raceWith(waitSeconds(1)),
                pickup.coralHold());
    }
}
