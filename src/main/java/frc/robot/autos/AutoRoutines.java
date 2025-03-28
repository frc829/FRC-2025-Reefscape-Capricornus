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
        autoChooser.addRoutine("Squirtle", this::squirtle);
        autoChooser.addRoutine("Grapploct", this::grapploct);
        autoChooser.addRoutine("1MTest", this::test1M);
        autoChooser.addRoutine("0MTest", this::test0M);
    }

    private AutoRoutine test1M(){
        AutoRoutine routine = factory.newRoutine("1MTest");
        AutoTrajectory trajectory = routine.trajectory("1MTest");
        Command routineCommand = sequence(trajectory.resetOdometry(), trajectory.cmd());
        routine.active().onTrue(routineCommand);
        return routine;
    }

    private AutoRoutine test0M(){
        AutoRoutine routine = factory.newRoutine("0MTest");
        AutoTrajectory trajectory = routine.trajectory("0MTest");
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
