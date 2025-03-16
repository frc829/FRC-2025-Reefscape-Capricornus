package frc.robot.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.CoralPickup;
import frc.robot.commands.CoralScore;

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
    private final CoralPickup coralPickup;
    private final CoralScore coralScore;

    public AutoRoutines(
            AutoFactory factory,
            CoralPickup coralPickup,
            CoralScore coralScore) {
        this.coralPickup = coralPickup;
        this.coralScore = coralScore;

        AutoChooser autoChooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        this.factory = factory;
        autoChooser.addRoutine("Noob S2", this::theNoobSpot);
        autoChooser.addRoutine("Noob S1", this::noobS1);
        autoChooser.addRoutine("Noob S1L4", this::noobS1L4);
        autoChooser.addRoutine("Noob S3", this::noobS3);
        autoChooser.addRoutine("S2 L4 Left", this::S2L4Left);
        autoChooser.addRoutine("TwoCoralS3", this::TwoCoralS3);
        autoChooser.addRoutine("ThreeCoralS3", this::ThreeCoralS3);
        autoChooser.addRoutine("FourCoralS3", this::FourCoralS3);
        // autoChooser.addRoutine("Plop and Shop", this::plopAndShop);
    }

    private AutoRoutine theNoobSpot() {
        AutoRoutine routine = factory.newRoutine("Noob S2");
        AutoTrajectory S2toG = routine.trajectory("S2-to-GH");
        Command routineCommand = sequence(S2toG.resetOdometry(), S2toG.cmd());
        routine.active().onTrue(routineCommand);
        S2toG.atTime("Align").onTrue(coralScore.l1Align());
        S2toG.done().onTrue(scoreL1());
        return routine;
    }

    private AutoRoutine noobS1() {
        AutoRoutine routine = factory.newRoutine("Noob S1");
        AutoTrajectory S2toEF = routine.trajectory("S1-to-EF");
        Command routineCommand = sequence(S2toEF.resetOdometry(), S2toEF.cmd());
        routine.active().onTrue(routineCommand);
        S2toEF.atTime("Align").onTrue(coralScore.l1Align());
        S2toEF.done().onTrue(scoreL1());
        return routine;
    }

    private AutoRoutine noobS1L4() {
        AutoRoutine routine = factory.newRoutine("Noob S1 L4");
        AutoTrajectory S2toEFL4 = routine.trajectory("S1-to-EF-L4");
        Command routineCommand = sequence(S2toEFL4.resetOdometry(), S2toEFL4.cmd());
        routine.active().onTrue(routineCommand);
        S2toEFL4.atTime("Align").onTrue(coralScore.l4Align());
        S2toEFL4.done().onTrue(scoreL4());
        return routine;
    }

    private AutoRoutine noobS3() {
        AutoRoutine routine = factory.newRoutine("Noob S3");
        AutoTrajectory S3toIJ = routine.trajectory("S3-to-IJ");
        Command routineCommand = sequence(S3toIJ.resetOdometry(), S3toIJ.cmd());
        routine.active().onTrue(routineCommand);
        S3toIJ.atTime("Align").onTrue(coralScore.l1Align());
        S3toIJ.done().onTrue(scoreL1());
        return routine;
    }

    private AutoRoutine S2L4Left() {
        AutoRoutine routine = factory.newRoutine("S2-L4-Left");
        AutoTrajectory traj = routine.trajectory("S2G-to-G");
        Command routineCommand = sequence(traj.resetOdometry(), traj.cmd());
        routine.active().onTrue(routineCommand);
        traj.atTime("Align").onTrue(coralScore.l4Align());
        traj.done().onTrue(waitSeconds(3.0).andThen(scoreL4()));
        return routine;
    }

    private AutoRoutine TwoCoralS3() {
        AutoRoutine routine = factory.newRoutine("Two-Coral-S3");
        AutoTrajectory traj0 = routine.trajectory("S1-to-EF-L4");
        AutoTrajectory traj1 = routine.trajectory("EF-to-SouthRight");
        AutoTrajectory traj2 = routine.trajectory("SouthRight-to-CD");
        Command cmd = sequence(traj0.resetOdometry(), traj0.cmd());
        routine.active().onTrue(cmd);

        // First Trajectory Score
        traj0.atTime("Align").onTrue(coralScore.l4Align());
        traj0.done().onTrue(scoreL4().withDeadline(waitSeconds(2.0)).andThen(traj1.spawnCmd()));


        // Second Trajectory Pickup
        traj1.atTime("Reset").onTrue(coralPickup.hardReset());
        traj1.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj1.done().onTrue(
                sequence(
                        waitSeconds(2),
                        coralPickup.holdFromBack().withDeadline(waitSeconds(0.25)).andThen(traj2.spawnCmd())));


        // Third Trajectory Score
        traj2.atTime("Reset").onTrue(coralPickup.holdFromBack());
        traj2.atTime("Align").onTrue(coralScore.l4Align());
        traj2.done().onTrue(
                sequence(
                        // coralScore.l4Align().withDeadline(waitSeconds(1.0)),
                        scoreL4().withDeadline(waitSeconds(2.0)))
        );

        return routine;

    }

    private AutoRoutine ThreeCoralS3() {
        AutoRoutine routine = factory.newRoutine("Two-Coral-S3");
        AutoTrajectory traj0 = routine.trajectory("S3-to-IJ-Fast");
        AutoTrajectory traj1 = routine.trajectory("IJ-to-NorthRight");
        AutoTrajectory traj2 = routine.trajectory("NorthRight-to-KL");
        AutoTrajectory traj3 = routine.trajectory("KL-to-NorthRight");
        AutoTrajectory traj4 = routine.trajectory("NorthRight-to-L");

        Command cmd = sequence(traj0.resetOdometry(), traj0.cmd());
        routine.active().onTrue(cmd);

        // First Trajectory Score
        traj0.atTime("Align").onTrue(coralScore.l4Align());
        traj0.done().onTrue(
                sequence(
                        race(scoreL4(), waitSeconds(0.25)),
                        race(coralPickup.hold(), waitSeconds(1))));
        traj0.doneFor(1.25).onTrue(traj1.cmd());


        // Second Trajectory Pickup
        traj1.atTime("Reset").onTrue(coralPickup.hardReset());
        traj1.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj1.done().onTrue(
                sequence(
                        waitSeconds(1),
                        race(coralPickup.holdFromBack(), waitSeconds(0.25))));

        traj1.doneFor(1.0).onTrue(traj2.cmd());

        // Third Trajectory Score
        traj2.atTime("Reset").onTrue(coralPickup.hold());
        traj2.atTime("Align").onTrue(coralScore.l4Align());
        traj2.done().onTrue(
                sequence(
                        waitSeconds(0.25),
                        race(scoreL4(), waitSeconds(0.25)),
                        race(coralPickup.hardReset(), waitSeconds(1.0))));
        traj2.doneFor(1.25).onTrue(traj3.cmd());


        // Fourth Trajectory Pickup
        traj3.atTime("Reset").onTrue(coralPickup.hardReset());
        traj3.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj3.done().onTrue(
                sequence(
                        waitSeconds(1),
                        race(coralPickup.holdFromBack(), waitSeconds(0.25))));
        traj3.doneFor(1.0).onTrue(traj4.cmd());


        // Fifth Trajectory Score
        traj4.atTime("Reset").onTrue(coralPickup.hold());
        traj4.atTime("Align").onTrue(coralScore.l4Align());
        traj4.done().onTrue(
                sequence(
                        waitSeconds(0.25),
                        race(scoreL4(), waitSeconds(0.25)),
                        race(coralPickup.hardReset(), waitSeconds(1.0))));

        return routine;

    }

    private AutoRoutine FourCoralS3() {
        AutoRoutine routine = factory.newRoutine("Two-Coral-S3");
        AutoTrajectory traj0 = routine.trajectory("S3-to-IJ-Fast");
        AutoTrajectory traj1 = routine.trajectory("IJ-to-NorthRight");
        AutoTrajectory traj2 = routine.trajectory("NorthRight-to-KL");
        AutoTrajectory traj3 = routine.trajectory("KL-to-NorthRight");
        AutoTrajectory traj4 = routine.trajectory("NorthRight-to-L");
        AutoTrajectory traj5 = routine.trajectory("L-to-NorthRight");
        AutoTrajectory traj6 = routine.trajectory("NorthRight-to-A");


        Command cmd = sequence(traj0.resetOdometry(), traj0.cmd());
        routine.active().onTrue(cmd);

        // First Trajectory Score
        traj0.atTime("Align").onTrue(coralScore.l4Align());
        traj0.done().onTrue(
                sequence(
                        race(scoreL4(), waitSeconds(0.25)),
                        race(coralPickup.hold(), waitSeconds(1))));
        traj0.doneFor(1.25).onTrue(traj1.cmd());


        // Second Trajectory Pickup
        traj1.atTime("Reset").onTrue(coralPickup.hardReset());
        traj1.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj1.done().onTrue(
                sequence(
                        waitSeconds(1),
                        race(coralPickup.holdFromBack(), waitSeconds(0.25))));

        traj1.doneFor(1.0).onTrue(traj2.cmd());

        // Third Trajectory Score
        traj2.atTime("Reset").onTrue(coralPickup.hold());
        traj2.atTime("Align").onTrue(coralScore.l4Align());
        traj2.done().onTrue(
                sequence(
                        waitSeconds(0.25),
                        race(scoreL4(), waitSeconds(0.25)),
                        race(coralPickup.hardReset(), waitSeconds(1.0))));
        traj2.doneFor(1.25).onTrue(traj3.cmd());


        // Fourth Trajectory Pickup
        traj3.atTime("Reset").onTrue(coralPickup.hardReset());
        traj3.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj3.done().onTrue(
                sequence(
                        waitSeconds(1),
                        race(coralPickup.holdFromBack(), waitSeconds(0.25))));
        traj3.doneFor(1.0).onTrue(traj4.cmd());


        // Fifth Trajectory Score
        traj4.atTime("Reset").onTrue(coralPickup.hold());
        traj4.atTime("Align").onTrue(coralScore.l4Align());
        traj4.done().onTrue(
                sequence(
                        waitSeconds(0.25),
                        race(scoreL4(), waitSeconds(0.25)),
                        race(coralPickup.hardReset(), waitSeconds(1.0))));
        traj4.doneFor(1.0).onTrue(traj5.cmd());

        // Sixth Trajectory Pickup
        traj5.atTime("Reset").onTrue(coralPickup.hardReset());
        traj5.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj5.done().onTrue(
                sequence(
                        waitSeconds(1),
                        race(coralPickup.holdFromBack(), waitSeconds(0.25))));
        traj5.doneFor(1.25).onTrue(traj6.cmd());

        // Seventh Trajectory Score
        traj6.atTime("Reset").onTrue(coralPickup.hold());
        traj6.atTime("Align").onTrue(coralScore.l4Align());
        traj6.done().onTrue(
                sequence(
                        waitSeconds(0.25),
                        race(scoreL4(), waitSeconds(0.25)),
                        race(coralPickup.hardReset(), waitSeconds(1.0))));

        return routine;

    }

    private Command scoreL1() {
        return sequence(
                coralScore.l1Score().withDeadline(waitSeconds(3)),
                coralPickup.hold())
                .withName("ScoreL1");

    }

    private Command scoreL4() {
        return sequence(
                waitSeconds(0.5),
                coralScore.l234Score(),
                waitSeconds(0.5),
                coralPickup.hold())
                .withName("ScoreL4");
    }
}
