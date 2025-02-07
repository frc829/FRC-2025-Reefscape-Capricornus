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
import frc.robot.subsystems.arm.CommandArmFactory;
import frc.robot.subsystems.elevator.CommandElevatorFactory;
import frc.robot.subsystems.arm.CommandArmConstants;
import frc.robot.subsystems.elevator.CommandElevatorConstants;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveConstants;
import frc.robot.routines.AutoRoutines;
import frc.robot.routines.DriverRoutines;
import frc.robot.routines.OperatorRoutines;
import frc.robot.subsystems.arm.CommandArm;
import frc.robot.subsystems.elevator.CommandElevator;
import frc.robot.subsystems.swerveDrive.CommandSwerveDrive;
import frc.robot.subsystems.swerveDrive.CommandSwerveFactory;

public class Robot extends TimedRobot {

    public Robot() {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController operatorController = new CommandXboxController(1);
        CommandSwerveDrive commandSwerveDrive = CommandSwerveDriveConstants.createCommandSwerve();
        commandSwerveDrive.configureAutoBuilder();
        CommandArm commandArm = CommandArmConstants.createCommandArm();
        CommandElevator commandElevator = CommandElevatorConstants.createCommandElevator();
        CommandArmFactory commandArmFactory = new CommandArmFactory(commandArm);
        CommandElevatorFactory commandElevatorFactory = new CommandElevatorFactory(commandElevator);
        CommandSwerveFactory commandSwerveFactory = new CommandSwerveFactory(commandSwerveDrive);
        new DriverRoutines(
                driverController,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null);
        new OperatorRoutines(
                operatorController,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null);
        AutoChooser autoChooser = new AutoChooser();
        AutoFactory autoFactory = commandSwerveDrive.createAutoFactory();
        new AutoRoutines(autoFactory, autoChooser);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
    }
}
