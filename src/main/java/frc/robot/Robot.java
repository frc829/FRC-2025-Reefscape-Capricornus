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
import frc.robot.commandFactories.AlgaePickupFactories;
import frc.robot.commandFactories.SubsystemCommandFactories;
import frc.robot.subsystems.algaeClaw.CommandAlgaeClaw;
import frc.robot.subsystems.algaeClaw.CommandAlgaeClawFactory;
import frc.robot.subsystems.arm.CommandArmFactory;
import frc.robot.subsystems.coralClaw.CommandCoralClawFactory;
import frc.robot.subsystems.dualIntake.CommandDualIntake;
import frc.robot.subsystems.dualIntake.CommandDualIntakeConstants;
import frc.robot.subsystems.dualIntake.CommandDualIntakeFactory;
import frc.robot.subsystems.elevator.CommandElevatorFactory;
import frc.robot.subsystems.arm.CommandArmConstants;
import frc.robot.subsystems.elevator.CommandElevatorConstants;
import frc.robot.subsystems.hook.CommandHookConstants;
import frc.robot.subsystems.hook.CommandHookFactory;
import frc.robot.subsystems.pneumatics.CommandPneumaticsConstants;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveConstants;
import frc.robot.routines.AutoRoutines;
import frc.robot.routines.DriverRoutines;
import frc.robot.routines.OperatorRoutines;
import frc.robot.subsystems.arm.CommandArm;
import frc.robot.subsystems.elevator.CommandElevator;
import frc.robot.subsystems.swerveDrive.CommandSwerveDrive;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveFactory;
import frc.robot.subsystems.winch.CommandWinch;
import frc.robot.subsystems.winch.CommandWinchConstants;
import frc.robot.subsystems.winch.CommandWinchFactory;
import frc.robot.subsystems.wrist.CommandWristConstants;
import frc.robot.subsystems.wrist.CommandWristFactory;

public class Robot extends TimedRobot {

    public Robot() {
        CommandSwerveDrive commandSwerveDrive = CommandSwerveDriveConstants.createCommandSwerve();
        commandSwerveDrive.configureAutoBuilder();
        SubsystemCommandFactories subsystemCommandFactories = new SubsystemCommandFactories(
                new CommandAlgaeClawFactory(CommandPneumaticsConstants.createCommandAlgaeClaw()),
                new CommandArmFactory(CommandArmConstants.createCommandArm()),
                new CommandCoralClawFactory(CommandPneumaticsConstants.createCommandCoralClaw()),
                new CommandDualIntakeFactory(CommandDualIntakeConstants.createCommandIntake()),
                new CommandElevatorFactory(CommandElevatorConstants.createCommandElevator()),
                new CommandHookFactory(CommandHookConstants.createCommandHook()),
                new CommandSwerveDriveFactory(commandSwerveDrive),
                new CommandWinchFactory(CommandWinchConstants.createCommandWinch()),
                new CommandWristFactory(CommandWristConstants.createCommandWrist()));
        AlgaePickupFactories algaePickupFactories = new AlgaePickupFactories(subsystemCommandFactories);
        new DriverRoutines(
                subsystemCommandFactories);
        new OperatorRoutines(
                subsystemCommandFactories,
                algaePickupFactories);
        AutoChooser autoChooser = new AutoChooser();
        AutoFactory autoFactory = commandSwerveDrive.createAutoFactory();
        new AutoRoutines(autoFactory, autoChooser);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
    }
}
