// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.controllers.ManualController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commandFactories.PickupFactories;
import frc.robot.commandFactories.ResetFactories;
import frc.robot.commandFactories.ScoringFactories;
import frc.robot.commandFactories.SubsystemCommandFactories;
import frc.robot.controllers.OperatorXboxController;
import frc.robot.routines.*;
import frc.robot.subsystems.dualIntake.DualIntakeSubsystemConstants;
import frc.robot.subsystems.arm.ArmSubsystemConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystemConstants;
import frc.robot.subsystems.pneumatics.PneumaticsSubsystemConstants;
import frc.robot.subsystems.power.PowerSubsystemConstants;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveConstants;
import frc.robot.subsystems.swerveDrive.CommandSwerveDrive;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveFactory;
import frc.robot.subsystems.winch.WinchSubsystemConstants;
import frc.robot.subsystems.wrist.WristSubsystemConstants;

public class Robit extends TimedRobot {

    public Robit() {
        PortForwarder.add(5800, "orangepi50.local", 5800);
        CanBridge.runTCP();
        CommandSwerveDrive commandSwerveDrive = CommandSwerveDriveConstants.createCommandSwerve();
        commandSwerveDrive.configureAutoBuilder();
        // DriverController driverController = new DriverController(Constants.controllerDeadband);
        OperatorXboxController operatorXboxController = new OperatorXboxController(Constants.controllerDeadband);
        // OperatorFlightStickController operatorFlightStickController = new OperatorFlightStickController(Constants.controllerDeadband);
        ManualController manualController = new ManualController(Constants.controllerDeadband);
        SubsystemCommandFactories subsystemCommandFactories = new SubsystemCommandFactories(
                PneumaticsSubsystemConstants.createAlgaeClaw(),
                ArmSubsystemConstants.create(),
                PneumaticsSubsystemConstants.createCoralClaw(),
                DualIntakeSubsystemConstants.create(),
                ElevatorSubsystemConstants.create(),
                PneumaticsSubsystemConstants.create(),
                PowerSubsystemConstants.create(),
                new CommandSwerveDriveFactory(commandSwerveDrive),
                WinchSubsystemConstants.create(),
                WristSubsystemConstants.create());
        PickupFactories pickupFactories = new PickupFactories(subsystemCommandFactories);
        ResetFactories resetFactories = new ResetFactories(subsystemCommandFactories);
        ScoringFactories scoringFactories = new ScoringFactories(subsystemCommandFactories, resetFactories);
        new PickupRoutines(operatorXboxController, pickupFactories, resetFactories);
        new ManualRoutines(subsystemCommandFactories, manualController);
        // new DriverRoutines(subsystemCommandFactories, driverController);
        // new ScoringRoutines(
        //         operatorXboxController,
        //         operatorFlightStickController,
        //         scoringFactories);
        AutoChooser autoChooser = new AutoChooser();
        AutoFactory autoFactory = commandSwerveDrive.createAutoFactory();
        new AutoRoutines(autoFactory, autoChooser);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        DataLogManager.start();
        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
    }
}
