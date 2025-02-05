package frc.robot.routines;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ComboCommandFactory;
import frc.robot.subsystems.CommandArm;
import frc.robot.subsystems.CommandElevator;
import frc.robot.subsystems.CommandSwerveDrive;

public class DriverRoutines {

    private final CommandXboxController driverController;
    private final ComboCommandFactory commandFactory;
    private final CommandSwerveDrive commandSwerveDrive;
    private final CommandArm commandArm;
    private final CommandElevator commandElevator;

    public DriverRoutines(CommandXboxController driverController,
                          ComboCommandFactory commandFactory,
                          CommandSwerveDrive commandSwerveDrive,
                          CommandArm commandArm,
                          CommandElevator commandElevator) {
        this.driverController = driverController;
        this.commandFactory = commandFactory;
        this.commandSwerveDrive = commandSwerveDrive;
        this.commandArm = commandArm;
        this.commandElevator = commandElevator;
        zeroWheels();
        brake();
        pointModules();
        seedFieldCentric();
        goToReef0();
        toggleClock();
    }

    private void zeroWheels() {
        // TODO: bind to driverController button either while or on True
    }

    private static void brake() {
        //  bind to driverController button either while or on True
    }

    private void pointModules() {
        // TODO: bind to driverController button either while or on True
    }

    private void seedFieldCentric() {
        // reset the field-centric heading on left bumper press
        // TODO: bind to driverController button either while or on True
    }

    private void goToReef0(){
        driverController.a().whileTrue(commandFactory.goToReef0());
    }


    private void toggleClock() {
        driverController.x().onTrue(commandSwerveDrive.toggleClock());
    }
}
