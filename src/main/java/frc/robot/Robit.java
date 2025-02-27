// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commandFactories.*;
import frc.robot.routines.*;
import frc.robot.subsystems.dualIntake.DualIntakeSubsystemConstants;
import frc.robot.subsystems.arm.ArmSubsystemConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystemConstants;
import frc.robot.subsystems.pneumatics.PneumaticsSubsystemConstants;
import frc.robot.subsystems.power.PowerSubsystemConstants;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;
import frc.robot.subsystems.winch.WinchSubsystemConstants;
import frc.robot.subsystems.wrist.WristSubsystemConstants;

public class Robit extends TimedRobot {

    public Robit() {
        SmartDashboard.putString("Robot Comments", Constants.robotComments);
        PortForwarder.add(5800, "orangepi50.local", 5800);
        CanBridge.runTCP();
        SwerveDriveSubsystem swerveDriveSubsystem = SwerveDriveSubsystemConstants.createCTRESwerveDrive();
        AutoFactory autoFactory = swerveDriveSubsystem.createAutoFactory();
        DrivingFactories driving = new DrivingFactories(swerveDriveSubsystem, autoFactory);
        ManipulatorFactories manipulator = new ManipulatorFactories(
                PneumaticsSubsystemConstants.createAlgaeClaw(),
                ArmSubsystemConstants.create(),
                PneumaticsSubsystemConstants.createCoralClaw(),
                DualIntakeSubsystemConstants.create(),
                ElevatorSubsystemConstants.create(),
                PneumaticsSubsystemConstants.create(),
                PowerSubsystemConstants.create(),
                WinchSubsystemConstants.create(),
                WristSubsystemConstants.create());
        PickupFactories pickupFactories = new PickupFactories(manipulator);
        ScoringFactories scoringFactories = new ScoringFactories(manipulator);
        ManualFactories manualFactories = new ManualFactories(manipulator);
        new TriggerMap(driving, pickupFactories, scoringFactories, manualFactories);
        new AutoRoutines(autoFactory, pickupFactories, scoringFactories);
        SmartDashboard.putData(CommandScheduler.getInstance());
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);
        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
    }
}
