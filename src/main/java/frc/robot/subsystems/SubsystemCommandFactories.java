package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.CommandArm;
import frc.robot.subsystems.elevator.CommandElevator;
import frc.robot.subsystems.swerveDrive.CommandSwerveDrive;

public class SubsystemCommandFactories {
    public final CommandXboxController driverController;
    public final CommandXboxController operatorController;
    public final CommandSwerveDrive commandSwerveDrive;
    public final CommandArm commandArm;
    public final CommandElevator commandElevator;

    public SubsystemCommandFactories(
            CommandXboxController driverController,
            CommandXboxController operatorController,
            CommandSwerveDrive commandSwerveDrive,
            CommandArm commandArm,
            CommandElevator commandElevator) {
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.commandSwerveDrive = commandSwerveDrive;
        this.commandArm = commandArm;
        this.commandElevator = commandElevator;
    }







}
