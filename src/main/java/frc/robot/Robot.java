// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.CommandArmConstants;
import frc.robot.constants.CommandElevatorConstants;
import frc.robot.constants.CommandSwerveDriveConstants;
import frc.robot.subsystems.CommandArm;
import frc.robot.subsystems.CommandElevator;
import frc.robot.subsystems.CommandSwerveDrive;

public class Robot extends TimedRobot {

    public Robot() {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController operatorController = new CommandXboxController(1);
        CommandSwerveDrive commandSwerveDrive = CommandSwerveDriveConstants.createCommandSwerve();
        commandSwerveDrive.configureAutoBuilder();
        CommandArm commandArm = CommandArmConstants.createCommandArm();
        CommandElevator commandElevator = CommandElevatorConstants.createCommandElevator();
        CommandFactory commandFactory = new CommandFactory(
                driverController,
                operatorController,
                commandSwerveDrive,
                commandArm,
                commandElevator);
        new DefaultRoutines(driverController,
                operatorController,
                commandFactory,
                commandSwerveDrive,
                commandArm,
                commandElevator);
        new DriverRoutines(
                driverController,
                commandFactory,
                commandSwerveDrive,
                commandArm,
                commandElevator);
        new OperatorRoutines(
                operatorController,
                commandFactory,
                commandArm,
                commandElevator);
        AutoChooser autoChooser = new AutoChooser();
        AutoFactory autoFactory = commandSwerveDrive.createAutoFactory();
        new AutoRoutines(autoFactory, autoChooser);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
    }
}
