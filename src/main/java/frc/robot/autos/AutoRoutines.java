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

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutoRoutines {


    private record Path(
            String trajectoryName,
            List<Supplier<Command>> events,
            Supplier<Command> done) {
    }

    private final AutoFactory factory;
    private final AlgaePickup algaePickup;
    private final AlgaeScore algaeScore;
    private final CoralPickup coralPickup;
    private final CoralScore coralScore;


    private final Path middle0L1;
    private final Path middle0L4;
    private final Path S10L1;
    private final Path S30L1;

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

        middle0L1 = new Path(
                "S2-to-GH-L1",
                List.of(() -> coralScore.l1Align().withName("Align")),
                coralScore::l1Score);
        middle0L4 = new Path(
                "S2-to-GH-L4",
                List.of(() -> coralScore.l4Align().withName("Align")),
                coralScore::l234Score);
        S10L1 = new Path(
                "S1-to-EF-L1",
                List.of(() -> coralScore.l1Align().withName("Align")),
                coralScore::l1Score);
        S30L1 = new Path(
                "S3-to-IJ-L1",
                List.of(() -> coralScore.l1Align().withName("Align")),
                coralScore::l1Score
        );
        autoChooser.addRoutine("Noob S2",
                () -> createRoutine(
                        "Noob S2",
                        middle0L1));
        autoChooser.addRoutine("The Flying Dutchman",
                () -> createRoutine(
                        "The Flying Dutchman",
                        middle0L4));
        autoChooser.addRoutine("Noob S1",
                () -> createRoutine(
                        "Noob S1",
                        S10L1));
        autoChooser.addRoutine("Noob S3",
                () -> createRoutine(
                        "Noob S1",
                        S30L1));

        autoChooser.addRoutine("Inteleon", this::inteleon);
        autoChooser.addRoutine("Squirtle", this::squirtle);
        autoChooser.addRoutine("IntelliJ", this::intelliJ);
        autoChooser.addRoutine("WarTortle", this::wartortle);
        autoChooser.addRoutine("Grapploct", this::grapploct);

    }

    private AutoRoutine createRoutine(String name, Path firstPath, Path... remainingPaths) {
        AutoRoutine routine = factory.newRoutine(name);
        AutoTrajectory trajectory0 = routine.trajectory(firstPath.trajectoryName);

        Command cmd = sequence(trajectory0.resetOdometry(), trajectory0.cmd());
        routine.active().onTrue(cmd);
        firstPath.events.forEach(event -> {
            Command eventCommand = event.get();
            trajectory0.atTime(eventCommand.getName()).onTrue(eventCommand);
        });
        trajectory0.done().onTrue(firstPath.done().get());

        Arrays.stream(remainingPaths)
                .forEach(path -> {
                    AutoTrajectory trajectory = routine.trajectory(path.trajectoryName);
                    path.events.forEach(event -> {
                        Command eventCommand = event.get();
                        trajectory.atTime(eventCommand.getName()).onTrue(eventCommand);
                    });
                    trajectory.done().onTrue(path.done().get());
                });

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
        traj0.done().onTrue(coralScore.l234Score().withDeadline(waitSeconds(2.0)).andThen(traj1.spawnCmd()));


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
                        coralScore.l234Score().withDeadline(waitSeconds(2.0))));

        return routine;
    }

    private AutoRoutine wartortle() {
        AutoRoutine routine = factory.newRoutine("WarTortle");
        AutoTrajectory traj0 = routine.trajectory(topTraj0Name);
        AutoTrajectory traj1 = routine.trajectory(topTraj1Name);
        AutoTrajectory traj2 = routine.trajectory(topTraj2Name);
        AutoTrajectory traj3 = routine.trajectory(topTraj3Name);
        AutoTrajectory traj4 = routine.trajectory(topTraj4Name);

        Command cmd = sequence(traj0.resetOdometry(), traj0.cmd());
        routine.active().onTrue(cmd);

        // First Trajectory Score
        traj0.atTime("Align").onTrue(coralScore.l4Align());
        traj0.done().onTrue(coralScore.l234Score().withDeadline(waitSeconds(0.75)).andThen(traj1.spawnCmd()));


        // Second Trajectory Pickup
        traj1.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj1.done().onTrue(
                sequence(
                        coralPickup.stationBack().until(coralPickup.hasCoral),
                        traj2.spawnCmd()));


        // Third Trajectory Score
        traj2.atTime("Align").onTrue(coralScore.l4Align());
        traj2.done().onTrue(
                sequence(
                        coralScore.l234Score().withDeadline(waitSeconds(0.75).andThen(traj3.spawnCmd()))));

        // Fourth Trajectory Score
        traj3.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj3.done().onTrue(
                sequence(
                        coralPickup.stationBack().until(coralPickup.hasCoral),
                        traj4.spawnCmd()));

        // Fifth Trajectory Score
        traj4.atTime("Align").onTrue(coralScore.l4Align());
        traj4.done().onTrue(
                sequence(
                        coralScore.l234Score().withDeadline(waitSeconds(0.75))));

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
        traj0.done().onTrue(coralScore.l234Score().withDeadline(waitSeconds(0.75)).andThen(traj1.spawnCmd()));


        // Second Trajectory Pickup
        traj1.atTime("Reset").onTrue(coralPickup.hardReset());
        traj1.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj1.done().onTrue(
                sequence(
                        coralPickup.stationBack().until(coralPickup.hasCoral),
                        traj2.spawnCmd()));


        // Third Trajectory Score
        traj2.atTime("Reset").onTrue(coralPickup.holdFromBack());
        traj2.atTime("Align").onTrue(coralScore.l4Align());
        traj2.done().onTrue(
                sequence(
                        coralScore.l234Score().withDeadline(waitSeconds(0.75).andThen(traj3.spawnCmd()))));

        // Fourth Trajectory Score
        traj3.atTime("Reset").onTrue(coralPickup.hardReset());
        traj3.atTime("Pickup").onTrue(coralPickup.stationBack());
        traj3.done().onTrue(
                sequence(
                        coralPickup.stationBack().until(coralPickup.hasCoral),
                        traj4.spawnCmd()));

        // Fifth Trajectory Score
        traj4.atTime("Reset").onTrue(coralPickup.holdFromBack());
        traj4.atTime("Align").onTrue(coralScore.l4Align());
        traj4.done().onTrue(
                sequence(
                        coralScore.l234Score().withDeadline(waitSeconds(0.75))));

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
        traj0.done().onTrue(coralScore.l234Score().withDeadline(waitSeconds(2.0)).andThen(traj1.spawnCmd()));


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
                        coralScore.l234Score().withDeadline(waitSeconds(2.0))));

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
                .andThen(coralScore.l234Score().withDeadline(waitSeconds(1.0)))
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

    private AutoRoutine theNoobSpot() {
        AutoRoutine routine = factory.newRoutine("Noob S2");
        AutoTrajectory S2toG = routine.trajectory("S2-to-GH-L1");
        Command routineCommand = sequence(S2toG.resetOdometry(), S2toG.cmd());
        routine.active().onTrue(routineCommand);
        S2toG.atTime("Align").onTrue(coralScore.l1Align());
        S2toG.done().onTrue(coralScore.l1Score());
        return routine;
    }

    private AutoRoutine noobS1() {
        AutoRoutine routine = factory.newRoutine("Noob S1");
        AutoTrajectory trajectory = routine.trajectory("S1-to-EF-L1");
        Command routineCommand = sequence(trajectory.resetOdometry(), trajectory.cmd());
        routine.active().onTrue(routineCommand);
        trajectory.atTime("Align").onTrue(coralScore.l1Align());
        trajectory.done().onTrue(coralScore.l1Score());
        return routine;
    }

    private AutoRoutine noobS3() {
        AutoRoutine routine = factory.newRoutine("Noob S3");
        AutoTrajectory S3toIJ = routine.trajectory("S3-to-IJ-L1");
        Command routineCommand = sequence(S3toIJ.resetOdometry(), S3toIJ.cmd());
        routine.active().onTrue(routineCommand);
        S3toIJ.atTime("Align").onTrue(coralScore.l1Align());
        S3toIJ.done().onTrue(coralScore.l1Score());
        return routine;
    }

    private AutoRoutine theFlyingDutchman() {
        AutoRoutine routine = factory.newRoutine("S2-L4-Left");
        AutoTrajectory traj = routine.trajectory("S2-to-GH-L4");
        Command routineCommand = sequence(traj.resetOdometry(), traj.cmd());
        routine.active().onTrue(routineCommand);

        traj.atTime("Align").onTrue(coralScore.l4Align());
        traj.done().onTrue(waitSeconds(1.0).andThen(coralScore.l234Score()));

        return routine;
    }

}
