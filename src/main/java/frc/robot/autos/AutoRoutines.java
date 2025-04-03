package frc.robot.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AlgaePickup;
import frc.robot.commands.AlgaeScore;
import frc.robot.commands.CoralPickup;
import frc.robot.commands.CoralScore;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutoRoutines {

    private final AutoFactory factory;
    private final AlgaePickup algaePickup;
    private final AlgaeScore algaeScore;
    private final CoralPickup coralPickup;
    private final CoralScore coralScore;

    private final String topTraj0Name = "Top01";
    private final String topTraj1Name = "Top02";
    private final String topTraj2Name = "Top03";
    private final String topTraj3Name = "Top04";
    private final String topTraj4Name = "Top05";

    public AutoRoutines(
            AutoFactory factory,
            AlgaePickup algaePickup,
            AlgaeScore algaeScore,
            CoralPickup coralPickup,
            CoralScore coralScore) {
        this.algaePickup = algaePickup;
        this.algaeScore = algaeScore;
        this.coralPickup = coralPickup;
        this.coralScore = coralScore;

        AutoChooser autoChooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        this.factory = factory;
        autoChooser.addRoutine("Noob S2", this::theNoobSpot);
        autoChooser.addRoutine("Noob S1", this::noobS1);
        autoChooser.addRoutine("Noob S3", this::noobS3);
        autoChooser.addRoutine("The Flying Dutchman", this::theFlyingDutchman);
        autoChooser.addRoutine("Inteleon", this::inteleon);
        autoChooser.addRoutine("IntelliJ", this::intelliJ);
        autoChooser.addRoutine("Squirtle", this::squirtle);
        autoChooser.addRoutine("Grapploct", this::grapploct);
        autoChooser.addRoutine("Kyogre", this::kyogre);
        autoChooser.addRoutine("Test1", this::test1);
        autoChooser.addRoutine("Test2", this::test2);
        autoChooser.addRoutine("WarTortle", this::wartortle);

    }

    private AutoRoutine test1() {
        AutoRoutine routine = factory.newRoutine("Test1");
        AutoTrajectory trajectory = routine.trajectory("Test1");
        Command routineCommand = sequence(trajectory.resetOdometry(), trajectory.cmd());
        routine.active().onTrue(routineCommand);
        return routine;
    }

    private AutoRoutine test2() {
        AutoRoutine routine = factory.newRoutine("Test2");
        AutoTrajectory trajectory = routine.trajectory("Test2");
        Command routineCommand = sequence(trajectory.resetOdometry(), trajectory.cmd());
        routine.active().onTrue(routineCommand);
        return routine;
    }

    private AutoRoutine theNoobSpot() {
        AutoRoutine routine = factory.newRoutine("Noob S2");
        AutoTrajectory S2toG = routine.trajectory("S2-to-GH-L1");
        Command routineCommand = sequence(S2toG.resetOdometry(), S2toG.cmd());
        routine.active().onTrue(routineCommand);
        S2toG.atTime("Align").onTrue(coralScore.l1Align());
        S2toG.done().onTrue(scoreL1());
        return routine;
    }

    private AutoRoutine noobS1() {
        AutoRoutine routine = factory.newRoutine("Noob S1");
        AutoTrajectory trajectory = routine.trajectory("S1-to-EF-L1");
        Command routineCommand = sequence(trajectory.resetOdometry(), trajectory.cmd());
        routine.active().onTrue(routineCommand);
        trajectory.atTime("Align").onTrue(coralScore.l1Align());
        trajectory.done().onTrue(scoreL1());
        return routine;
    }

    private AutoRoutine noobS3() {
        AutoRoutine routine = factory.newRoutine("Noob S3");
        AutoTrajectory S3toIJ = routine.trajectory("S3-to-IJ-L1");
        Command routineCommand = sequence(S3toIJ.resetOdometry(), S3toIJ.cmd());
        routine.active().onTrue(routineCommand);
        S3toIJ.atTime("Align").onTrue(coralScore.l1Align());
        S3toIJ.done().onTrue(scoreL1());
        return routine;
    }

    private AutoRoutine grapploct() {
        AutoRoutine routine = factory.newRoutine("S2-L4-Left");
        AutoTrajectory traj0 = routine.trajectory("S2-to-GH-L4");
        AutoTrajectory traj1 = routine.trajectory("S2-Algae-Backup");
        AutoTrajectory traj2 = routine.trajectory("GetAlgae");
        AutoTrajectory traj3 = routine.trajectory("AlgaeBackup");
        AutoTrajectory traj4 = routine.trajectory("PROC");
        Command routineCommand = sequence(traj0.resetOdometry(), traj0.cmd());
        routine.active().onTrue(routineCommand);

        traj0.atTime("Align").onTrue(coralScore.l4Align());
        traj1.atTime("Align").onTrue(algaePickup.L2());
        traj2.atTime("Align").onTrue(algaePickup.L2());
        traj4.atTime("Proc").onTrue(algaePickup.hold());

        traj0.done().onTrue(waitSeconds(1.0)
                .andThen(scoreL4().withDeadline(waitSeconds(1.0)))
                .andThen(traj1.spawnCmd()));
        traj1.done().onTrue(
                sequence(
                        waitSeconds(1.5),
                        traj2.spawnCmd()
                ));
        traj2.done().onTrue(
                sequence(
                        traj3.spawnCmd()
                ));
        traj3.done().onTrue(
                sequence(
                        traj4.spawnCmd()
                ));
        traj4.done().onTrue(algaeScore.score());


        return routine;
    }

    private AutoRoutine kyogre() {
        AutoRoutine routine = factory.newRoutine("S2-L4-Left");
        AutoTrajectory traj0 = routine.trajectory("S2-to-GH-L4");
        AutoTrajectory traj1 = routine.trajectory("S2-Algae-Backup");
        AutoTrajectory traj2 = routine.trajectory("GetAlgae");
        AutoTrajectory traj3 = routine.trajectory("AlgaeBackup");
        AutoTrajectory traj4 = routine.trajectory("PROC");
        AutoTrajectory traj5 = routine.trajectory("NextAlgae");
        AutoTrajectory traj6 = routine.trajectory("Algae2");
        AutoTrajectory traj7 = routine.trajectory("PROC2");
        Command routineCommand = sequence(traj0.resetOdometry(), traj0.cmd());
        routine.active().onTrue(routineCommand);

        traj0.atTime("Align").onTrue(coralScore.l4Align());
        traj1.atTime("Align").onTrue(algaePickup.L2());
        traj2.atTime("Align").onTrue(algaePickup.L2());
        traj4.atTime("Proc").onTrue(algaePickup.hold());
        traj5.atTime("Algae").onTrue(algaePickup.L3());
        traj6.atTime("Grab").onTrue(algaePickup.L3());
        traj7.atTime("Proc").onTrue(algaePickup.hold());

        traj0.done().onTrue(waitSeconds(0.0)
                .andThen(scoreL4().withDeadline(waitSeconds(0.5)))
                .andThen(traj1.spawnCmd()));
        traj1.done().onTrue(
                sequence(
                        waitSeconds(1),
                        traj2.spawnCmd()
                ));
        traj2.done().onTrue(
                sequence(
                        traj3.spawnCmd()
                ));
        traj3.done().onTrue(
                sequence(
                        traj4.spawnCmd()
                ));
        traj4.done().onTrue(
                sequence(
                        algaeScore.score().withDeadline(waitSeconds(0.5)),
                        traj5.spawnCmd()
                ));
        traj5.done().onTrue(traj6.spawnCmd());
        traj6.done().onTrue(traj7.spawnCmd());
        traj7.done().onTrue(algaeScore.score().withDeadline(waitSeconds(1.25)));


        return routine;
    }

    private AutoRoutine theFlyingDutchman() {
        AutoRoutine routine = factory.newRoutine("S2-L4-Left");
        AutoTrajectory traj = routine.trajectory("S2-to-GH-L4");
        Command routineCommand = sequence(traj.resetOdometry(), traj.cmd());
        routine.active().onTrue(routineCommand);

        traj.atTime("Align").onTrue(coralScore.l4Align());
        traj.done().onTrue(waitSeconds(1.0).andThen(scoreL4()));

        return routine;
    }

    private AutoRoutine inteleon() {
        AutoRoutine routine = factory.newRoutine("Inteleon");
        AutoTrajectory traj0 = routine.trajectory("Bottom01");
        AutoTrajectory traj1 = routine.trajectory("Bottom02");
        AutoTrajectory traj2 = routine.trajectory("Bottom03");
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
                        coralPickup.stationBack().until(coralPickup.hasCoral),
                        // waitSeconds(1.0),
                        coralPickup.holdFromBack().withDeadline(waitSeconds(0.25)).andThen(traj2.spawnCmd())));


        // Third Trajectory Score
        traj2.atTime("Reset").onTrue(coralPickup.holdFromBack());
        traj2.atTime("Align").onTrue(coralScore.l4Align());
        traj2.done().onTrue(
                sequence(
                        coralScore.l4Align().withDeadline(waitSeconds(1.0)),
                        scoreL4().withDeadline(waitSeconds(2.0))));

        return routine;
    }

    private AutoRoutine auto(String name, String... args) {
        return null;
    }

    private AutoRoutine wartortle() {
        AutoRoutine routine = factory.newRoutine("Wartortle");
        AutoTrajectory traj0 = routine.trajectory("Top01");
        AutoTrajectory traj1 = routine.trajectory("Top02");
        AutoTrajectory traj2 = routine.trajectory("Top03");
        AutoTrajectory traj3 = routine.trajectory("Top04");
        AutoTrajectory traj4 = routine.trajectory("Top05");

        Command cmd = sequence(traj0.resetOdometry(), traj0.cmd());
        routine.active().onTrue(cmd);

        // First Trajectory Score
        traj0.atTime("Align").onTrue(coralScore.l4Align());
        traj0.done().onTrue(scoreL4().withDeadline(waitSeconds(0.75)).andThen(traj1.spawnCmd()));


        // Second Trajectory Pickup
        traj1.atTime("Reset").onTrue(coralPickup.hardReset());
        traj1.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj1.done().onTrue(
                sequence(
                        coralPickup.stationBack().until(coralPickup.hasCoral).raceWith(waitSeconds(1.5)),
                        traj2.spawnCmd()));


        // Third Trajectory Score
        traj2.atTime("Align").onTrue(coralScore.l4Align());
        traj2.done().onTrue(
                sequence(
                        scoreL4().withDeadline(waitSeconds(0.75).andThen(traj3.spawnCmd()))));

        // Fourth Trajectory Score
        traj3.atTime("Reset").onTrue(coralPickup.hardReset());
        traj3.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj3.done().onTrue(
                sequence(
                        coralPickup.stationBack().until(coralPickup.hasCoral).raceWith(waitSeconds(1.5)),
                        traj4.spawnCmd()));

        // Fifth Trajectory Score
        traj4.atTime("Align").onTrue(coralScore.l4Align());
        traj4.done().onTrue(
                sequence(
                        scoreL4().withDeadline(waitSeconds(0.75))));

        return routine;
    }

    private AutoRoutine intelliJ() {
        AutoRoutine routine = factory.newRoutine("IntelliJ");
        AutoTrajectory traj0 = routine.trajectory("Bottom01-3");
        AutoTrajectory traj1 = routine.trajectory("Bottom02-3");
        AutoTrajectory traj2 = routine.trajectory("Bottom03-3");
        AutoTrajectory traj3 = routine.trajectory("Bottom04-3");
        AutoTrajectory traj4 = routine.trajectory("Bottom05-3");

        Command cmd = sequence(traj0.resetOdometry(), traj0.cmd());
        routine.active().onTrue(cmd);

        // First Trajectory Score
        traj0.atTime("Align").onTrue(coralScore.l4Align());
        traj0.done().onTrue(scoreL4().withDeadline(waitSeconds(0.75)).andThen(traj1.spawnCmd()));


        // Second Trajectory Pickup
        traj1.atTime("Reset").onTrue(coralPickup.hardReset());
        traj1.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj1.done().onTrue(
                sequence(
                        coralPickup.stationBack().until(coralPickup.hasCoral).raceWith(waitSeconds(1.5)),
                        traj2.spawnCmd()));


        // Third Trajectory Score
        traj2.atTime("Align").onTrue(coralScore.l4Align());
        traj2.done().onTrue(
                sequence(
                        scoreL4().withDeadline(waitSeconds(0.75).andThen(traj3.spawnCmd()))));

        // Fourth Trajectory Score
        traj3.atTime("Reset").onTrue(coralPickup.hardReset());
        traj3.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj3.done().onTrue(
                sequence(
                        coralPickup.stationBack().until(coralPickup.hasCoral).raceWith(waitSeconds(1.5)),
                        traj4.spawnCmd()));

        // Fifth Trajectory Score
        traj4.atTime("Align").onTrue(coralScore.l4Align());
        traj4.done().onTrue(
                sequence(
                        scoreL4().withDeadline(waitSeconds(0.75))));

        return routine;
    }

    private AutoRoutine squirtle() {
        AutoRoutine routine = factory.newRoutine("Squirtle");
        AutoTrajectory traj0 = routine.trajectory("S3-to-IJ-L4");
        AutoTrajectory traj1 = routine.trajectory("IJ-to-NorthRight");
        AutoTrajectory traj2 = routine.trajectory("NorthRight-to-KL");
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
                        waitSeconds(1.0),
                        coralPickup.holdFromBack().withDeadline(waitSeconds(0.25)).andThen(traj2.spawnCmd())));


        // Third Trajectory Score
        traj2.atTime("Reset").onTrue(coralPickup.holdFromBack());
        traj2.atTime("Align").onTrue(coralScore.l4Align());
        traj2.done().onTrue(
                sequence(
                        coralScore.l4Align().withDeadline(waitSeconds(1.0)),
                        scoreL4().withDeadline(waitSeconds(2.0))));

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
