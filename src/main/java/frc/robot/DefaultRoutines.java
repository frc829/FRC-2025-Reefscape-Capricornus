package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandArm;
import frc.robot.subsystems.CommandElevator;
import frc.robot.subsystems.CommandSwerveDrive;

public class DefaultRoutines {

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final CommandFactory commandFactory;

    public DefaultRoutines(CommandXboxController driverController,
                           CommandXboxController operatorController,
                           CommandFactory commandFactory,
                           CommandSwerveDrive commandSwerveDrive,
                           CommandArm commandArm,
                           CommandElevator commandElevator) {
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.commandFactory = commandFactory;
        commandSwerveDrive.setDefaultCommand(commandFactory.defaultDrive());
        commandArm.setDefaultCommand(commandFactory.defaultArm());
    }
}
