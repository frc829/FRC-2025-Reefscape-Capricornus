package frc.robot.routines;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commandFactories.ManualFactories;
import frc.robot.commandFactories.PickupFactories;
import frc.robot.commandFactories.ScoringFactories;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(
            AutoFactory factory,
            PickupFactories pickup,
            ScoringFactories score) {
        factory
                .bind("coralFloor", pickup.coralFloor());

        AutoChooser autoChooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        this.factory = factory;
//        autoChooser.addRoutine("The Love Boat", this::theLoveBoatAuto);
    }

    private AutoRoutine theLoveBoatAuto() {
        final AutoRoutine routine = factory.newRoutine("The Love Boat Auto");
        final AutoTrajectory S3toG = routine.trajectory("S3toG");
        final AutoTrajectory GtoTopStation = routine.trajectory("GtoTopStation");
        routine.active().onTrue(
                S3toG.resetOdometry()
                        .andThen(S3toG.cmd())
                        .andThen(Commands.print("SCORE L4"))
                        .andThen(Commands.waitSeconds(2))
                        .andThen(GtoTopStation.cmd()));
        return routine;
    }
}
