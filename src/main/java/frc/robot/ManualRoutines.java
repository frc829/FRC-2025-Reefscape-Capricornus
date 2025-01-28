package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ManualRoutines {

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    public ManualRoutines(
            CommandXboxController driverController,
            CommandXboxController operatorController,
            CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    }
}
