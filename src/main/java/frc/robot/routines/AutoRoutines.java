package frc.robot.routines;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory, AutoChooser autoChooser) {
        this.factory = factory;
        autoChooser.addRoutine("The Love Boat", this::theLoveBoatAuto);
    }

    private AutoRoutine theLoveBoatAuto() {
        final AutoRoutine routine = factory.newRoutine("The Love Boat Auto");
        final AutoTrajectory simplePath = routine.trajectory("The Love Boat");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }
}
