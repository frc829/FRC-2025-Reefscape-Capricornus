// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.CommandArmConstants;
import frc.robot.constants.CommandElevatorConstants;
import frc.robot.constants.CommandHookConstants;
import frc.robot.constants.CommandSwerveDriveConstants;
import frc.robot.subsystems.CommandArm;
import frc.robot.subsystems.CommandElevator;
import frc.robot.subsystems.CommandSwerveDrive;

public class RobotContainer {

    static final CommandXboxController driverController = new CommandXboxController(0);
    static final CommandXboxController operatorController = new CommandXboxController(1);
    static final CommandSwerveDrive commandSwerveDrive = CommandSwerveDriveConstants.createCommandSwerve();
    static final CommandArm commandArm = CommandArmConstants.createCommandArm();
    static final CommandElevator commandElevator = CommandElevatorConstants.createCommandElevator();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        commandSwerveDrive.configureAutoBuilder();
        autoFactory = commandSwerveDrive.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        DefaultRoutines.bind();
        DriverRoutines.bind();
        OperatorRoutines.bind();
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
