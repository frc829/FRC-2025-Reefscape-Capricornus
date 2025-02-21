package frc.robot.routines;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commandFactories.PickupFactories;
import frc.robot.commandFactories.ScoringFactories;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutoRoutines {



    private final AutoFactory factory;

    public AutoRoutines(
            AutoFactory factory,
            PickupFactories pickup,
            ScoringFactories score) {

        AutoChooser autoChooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        this.factory = factory;
        autoChooser.addRoutine("The Noob Spot", this::theNoobSpot);
    }

    private AutoRoutine theNoobSpot() {
        AutoRoutine routine = factory.newRoutine("The Noob Spot");
        AutoTrajectory S2toG = routine.trajectory("S2-to-G");

        routine.active().onTrue(
                sequence(
                        S2toG.resetOdometry(),
                        S2toG.cmd()));



        return routine;

    }
}
