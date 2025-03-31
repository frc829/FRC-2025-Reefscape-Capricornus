package frc.robot.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.RobotBase;
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
import java.util.stream.IntStream;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutoRoutines {


    private record Path(String trajectoryName, List<Supplier<Command>> events, Supplier<Command> done) {
    }


    private final AutoFactory factory;
    private final AlgaePickup algaePickup;
    private final AlgaeScore algaeScore;
    private final CoralPickup coralPickup;
    private final CoralScore coralScore;
    private final AutoChooser autoChooser;


    public AutoRoutines(AutoFactory factory, AlgaePickup algaePickup, AlgaeScore algaeScore, CoralPickup coralPickup, CoralScore coralScore) {
        this.factory = factory;
        this.algaePickup = algaePickup;
        this.algaeScore = algaeScore;
        this.coralPickup = coralPickup;
        this.coralScore = coralScore;
        autoChooser = new AutoChooser();

        // One Coral Autos
        addNoobS1();
        addTheFlyingDutchman();
        addNoobS3();

        // Two Coral Autos
        addInteleon();
        addSquirtle();

        // Three Coral Autos
        addIntelliJ();
        addWartortle();

        autoChooser.addRoutine("Grapploct", this::grapploct);


        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private void addNoobS1() {
        Path oneCoral_L4_S1_Start = new Path(
                "Bottom01",
                List.of(() -> coralScore.l4Align().withName("Align")),
                coralScore::l234Score);
        autoChooser.addRoutine(
                "Noob S1",
                () -> createRoutine(
                        "Noob S1",
                        oneCoral_L4_S1_Start));
    }

    private void addTheFlyingDutchman() {
        Path oneCoral_L4_S2_Start = new Path(
                "Middle01",
                List.of(() -> coralScore.l4Align().withName("Align")),
                coralScore::l234Score);
        autoChooser.addRoutine(
                "The Flying Dutchman",
                () -> createRoutine(
                        "The Flying Dutchman",
                        oneCoral_L4_S2_Start));
    }

    private void addNoobS3() {
        Path oneCoral_L4_S3_Start = new Path(
                "Top01",
                List.of(() -> coralScore.l4Align().withName("Align")),
                coralScore::l234Score);
        autoChooser.addRoutine(
                "Noob S3",
                () -> createRoutine(
                        "Noob S3",
                        oneCoral_L4_S3_Start));
    }

    private void addInteleon() {
        Path twoCoral_L4_S1_Start_0 = new Path(
                "Bottom01",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        Path twoCoral_L4_S1_Start_1 = new Path(
                "Bottom02",
                List.of(() -> coralPickup.stationBack().withName("Pickup")),
                () -> RobotBase.isReal()
                        ? coralPickup.stationBack().until(coralPickup.hasCoral)
                        : waitSeconds(0.5));
        Path twoCoral_L4_S1_Start_2 = new Path(
                "Bottom03",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        autoChooser.addRoutine(
                "Inteleon",
                () -> createRoutine(
                        "Inteleon",
                        twoCoral_L4_S1_Start_0,
                        twoCoral_L4_S1_Start_1,
                        twoCoral_L4_S1_Start_2));
    }

    private void addSquirtle() {
        Path twoCoral_L4_S3_Start_0 = new Path(
                "Top01",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        Path twoCoral_L4_S3_Start_1 = new Path(
                "Top02",
                List.of(() -> coralPickup.stationBack().withName("Pickup")),
                () -> RobotBase.isReal()
                        ? coralPickup.stationBack().until(coralPickup.hasCoral)
                        : waitSeconds(0.5));
        Path twoCoral_L4_S3_Start_2 = new Path(
                "Top03",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        autoChooser.addRoutine(
                "Squirtle",
                () -> createRoutine(
                        "Squirtle",
                        twoCoral_L4_S3_Start_0,
                        twoCoral_L4_S3_Start_1,
                        twoCoral_L4_S3_Start_2));
    }

    private void addIntelliJ() {
        Path threeCoral_L4_S1_Start_0 = new Path(
                "Bottom01",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        Path threeCoral_L4_S1_Start_1 = new Path(
                "Bottom02",
                List.of(() -> coralPickup.stationBack().withName("Pickup")),
                () -> RobotBase.isReal()
                        ? coralPickup.stationBack().until(coralPickup.hasCoral)
                        : waitSeconds(0.5));
        Path threeCoral_L4_S1_Start_2 = new Path(
                "Bottom03",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        Path threeCoral_L4_S1_Start_3 = new Path(
                "Bottom04",
                List.of(() -> coralPickup.stationBack().withName("Pickup")),
                () -> RobotBase.isReal()
                        ? coralPickup.stationBack().until(coralPickup.hasCoral)
                        : waitSeconds(0.5));
        Path threeCoral_L4_S1_Start_4 = new Path(
                "Bottom05",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        autoChooser.addRoutine(
                "IntelliJ",
                () -> createRoutine(
                        "IntelliJ",
                        threeCoral_L4_S1_Start_0,
                        threeCoral_L4_S1_Start_1,
                        threeCoral_L4_S1_Start_2,
                        threeCoral_L4_S1_Start_3,
                        threeCoral_L4_S1_Start_4));
    }

    private void addWartortle() {
        Path threeCoral_L4_S3_Start_0 = new Path(
                "Bottom01",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        Path threeCoral_L4_S3_Start_1 = new Path(
                "Bottom02",
                List.of(() -> coralPickup.stationBack().withName("Pickup")),
                () -> RobotBase.isReal()
                        ? coralPickup.stationBack().until(coralPickup.hasCoral)
                        : waitSeconds(0.5));
        Path threeCoral_L4_S3_Start_2 = new Path(
                "Bottom03",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        Path threeCoral_L4_S3_Start_3 = new Path(
                "Bottom04",
                List.of(() -> coralPickup.stationBack().withName("Pickup")),
                () -> RobotBase.isReal()
                        ? coralPickup.stationBack().until(coralPickup.hasCoral)
                        : waitSeconds(0.5));
        Path threeCoral_L4_S3_Start_4 = new Path(
                "Bottom05",
                List.of(() -> coralScore.l4Align().withName("Align")),
                () -> coralScore.l234Score().withDeadline(waitSeconds(1.0)));
        autoChooser.addRoutine(
                "Wartortle",
                () -> createRoutine(
                        "Wartortle",
                        threeCoral_L4_S3_Start_0,
                        threeCoral_L4_S3_Start_1,
                        threeCoral_L4_S3_Start_2,
                        threeCoral_L4_S3_Start_3,
                        threeCoral_L4_S3_Start_4));
    }

    private AutoRoutine createRoutine(String name, Path... paths) {
        AutoRoutine routine = factory.newRoutine(name);
        AutoTrajectory[] autoTrajectories = Arrays.stream(paths).map(path -> routine.trajectory(path.trajectoryName)).toArray(AutoTrajectory[]::new);

        IntStream.range(0, paths.length).forEachOrdered(i -> {
            if (i == 0) {
                Command commandOnActive = sequence(autoTrajectories[0].resetOdometry(), autoTrajectories[0].cmd());
                routine.active().onTrue(commandOnActive);
            }
            paths[i].events.forEach(event -> {
                Command eventCommand = event.get();
                autoTrajectories[i].atTime(eventCommand.getName()).onTrue(eventCommand);
            });
            if (paths[i].done != null) {
                autoTrajectories[i].done().onTrue(i < paths.length - 1
                        ? paths[i].done().get().andThen(autoTrajectories[i + 1].spawnCmd())
                        : paths[i].done().get());
            }
        });

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

        traj0.done().onTrue(waitSeconds(1.0).andThen(coralScore.l234Score().withDeadline(waitSeconds(1.0))).andThen(traj1.spawnCmd()));
        traj1.done().onTrue(sequence(waitSeconds(1.5), traj2.spawnCmd()));
        traj2.done().onTrue(sequence(traj3.spawnCmd()));
        traj3.done().onTrue(sequence(traj4.spawnCmd()));
        traj4.done().onTrue(algaeScore.score());


        return routine;
    }
}
