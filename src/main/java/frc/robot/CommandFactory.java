package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CommandFactory {

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final CommandSwerveDrivetrain drivetrain;

    public CommandFactory(
            CommandXboxController driverController,
            CommandXboxController operatorController,
            CommandSwerveDrivetrain drivetrain) {
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.drivetrain = drivetrain;
    }

}
