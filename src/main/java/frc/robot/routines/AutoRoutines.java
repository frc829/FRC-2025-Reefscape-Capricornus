package frc.robot.routines;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory, AutoChooser autoChooser) {
        this.factory = factory;
        autoChooser.addRoutine("SimplePath", this::simplePathAuto);
        autoChooser.addRoutine("TheLoveBoat", this::theLoveBoat);
    }

    private AutoRoutine simplePathAuto() {
        final AutoRoutine routine = factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    private AutoRoutine theLoveBoat() {
        final AutoRoutine routine = factory.newRoutine("The Love Boat Auto");
        final AutoTrajectory simplePath = routine.trajectory("The Love Boat");

        routine.active().onTrue(
                simplePath.resetOdometry()
                        .andThen(simplePath.cmd())
        );
        return routine;

    }
}
