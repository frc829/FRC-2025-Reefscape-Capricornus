package frc.robot.routines;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory, AutoChooser autoChooser) {
        this.factory = factory;
        autoChooser.addRoutine("The Love Boat", this::theLoveBoatAuto);
    }

    private AutoRoutine theLoveBoatAuto() {
        final AutoRoutine routine = factory.newRoutine("The Love Boat Auto");
        final AutoTrajectory S3toG = routine.trajectory("S3toG");
        final AutoTrajectory GtoTopStation = routine.trajectory("GtoTopStation");

        routine.active().onTrue(
                S3toG.cmd()
                        .andThen(S3toG.cmd())
                        .andThen(Commands.print("SCORE L4"))
                        .andThen(Commands.waitSeconds(2))
                        .andThen(GtoTopStation.cmd())
        );
        return routine;
    }
}
