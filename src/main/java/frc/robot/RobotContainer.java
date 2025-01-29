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

import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.CommandSwerveDrive;

public class RobotContainer {

    static final CommandXboxController driverController = new CommandXboxController(0);
    static final CommandXboxController operatorController = new CommandXboxController(1);
    static final CommandSwerveDrive commandSwerveDrive = SwerveDriveConstants.createCommandSwerve();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoFactory = commandSwerveDrive.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        DefaultRoutines.bind();
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
