// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//This is our code after the Mishiwaka 2025 Competition
package frc.robot;

import au.grapplerobotics.CanBridge;
import choreo.auto.AutoFactory;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.game.*;
import frc.robot.commands.system.*;
import frc.robot.autos.*;
import frc.robot.subsystems.dualIntake.DualIntakeSubsystemConstants;
import frc.robot.subsystems.arm.ArmSubsystemConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystemConstants;
import frc.robot.subsystems.pneumatics.PneumaticsSubsystemConstants;
import frc.robot.subsystems.power.PowerSubsystemConstants;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;
import frc.robot.subsystems.winch.WinchSubsystemConstants;
import frc.robot.subsystems.wrist.WristSubsystemConstants;
import frc.robot.triggermaps.*;

public class Robit extends TimedRobot {

    public Robit() {
        SmartDashboard.putString("Robot Comments", Constants.robotComments);
        PortForwarder.add(5800, "orangepi50.local", 5800);
        CanBridge.runTCP();

        double deadband = 0.05;
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController operatorController = new CommandXboxController(1);
        CommandJoystick climberController = new CommandJoystick(2);
        CommandXboxController manualController = new CommandXboxController(3);


        SwerveDriveSubsystem swerveDriveSubsystem = SwerveDriveSubsystemConstants.createCTRESwerveDrive();
        AutoFactory autoFactory = swerveDriveSubsystem.createAutoFactory();

        Drive drive = new Drive(swerveDriveSubsystem, autoFactory);
        Manipulator manipulator = new Manipulator(
                PneumaticsSubsystemConstants.createAlgaeClaw(),
                ArmSubsystemConstants.create(),
                PneumaticsSubsystemConstants.createCoralClaw(),
                DualIntakeSubsystemConstants.create(),
                ElevatorSubsystemConstants.create(),
                PneumaticsSubsystemConstants.create(),
                PowerSubsystemConstants.create(),
                WristSubsystemConstants.create());
        Climber climber = new Climber(WinchSubsystemConstants.create());

        AlgaePickup algaePickup = new AlgaePickup(manipulator);
        AlgaeScore algaeScore = new AlgaeScore(manipulator);
        CoralPickup coralPickup = new CoralPickup(manipulator);
        CoralScore coralScore = new CoralScore(manipulator);
        Manual manual = new Manual(manipulator);

        new DriveMap(driverController, deadband, drive);
        new AlgaePickupMap(operatorController, deadband, algaePickup);
        new AlgaeScoreMap(driverController, operatorController, deadband, algaeScore, coralPickup::hold);
        new CoralPickupMap(operatorController, coralPickup);
        new CoralScoreMap(driverController, operatorController, coralScore, coralPickup::hold);
        new ClimberMap(climberController, deadband, climber);
        new ManualMap(manualController, operatorController, deadband, manual);
        new AutoRoutines(autoFactory, coralPickup, coralScore);
        SmartDashboard.putData(CommandScheduler.getInstance());
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopPeriodic() {
    }
}
