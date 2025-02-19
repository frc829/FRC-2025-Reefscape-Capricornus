// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commandFactories.*;
import frc.robot.routines.*;
import frc.robot.subsystems.dualIntake.DualIntakeSubsystemConstants;
import frc.robot.subsystems.arm.ArmSubsystemConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystemConstants;
import frc.robot.subsystems.pneumatics.PneumaticsSubsystemConstants;
import frc.robot.subsystems.power.PowerSubsystemConstants;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;
import frc.robot.subsystems.wrist.WristSubsystemConstants;

public class Robit extends TimedRobot {

    public Robit() {
        PortForwarder.add(5800, "orangepi50.local", 5800);
        CanBridge.runTCP();
        SwerveDriveSubsystem swerveDriveSubsystem = SwerveDriveSubsystemConstants.create();
        DrivingFactories driving = new DrivingFactories(swerveDriveSubsystem);
        ManipulatorFactories manipulator = new ManipulatorFactories(
                PneumaticsSubsystemConstants.createAlgaeClaw(),
                ArmSubsystemConstants.create(),
                PneumaticsSubsystemConstants.createCoralClaw(),
                DualIntakeSubsystemConstants.create(),
                ElevatorSubsystemConstants.create(),
                PneumaticsSubsystemConstants.create(),
                PowerSubsystemConstants.create(),
                // WinchSubsystemConstants.create(),
                WristSubsystemConstants.create());
        PickupFactories pickupFactories = new PickupFactories(manipulator);
        ScoringFactories scoringFactories = new ScoringFactories(manipulator);
        ManualFactories manualFactories = new ManualFactories(manipulator);
        new TriggerMap(driving, pickupFactories, scoringFactories, manualFactories);

        AutoChooser autoChooser = new AutoChooser();
        AutoFactory autoFactory = swerveDriveSubsystem.createAutoFactory();
        new AutoRoutines(autoFactory, autoChooser);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        DataLogManager.start();
        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
    }
}
