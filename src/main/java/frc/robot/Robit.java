// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import choreo.auto.AutoFactory;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.AutoRoutines;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.ArmSubsystemConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystemConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystemConstants;
import frc.robot.subsystems.intakeWheel.AlgaeIntakeSubsystemConstants;
import frc.robot.subsystems.intakeWheel.CoralIntakeSubsystemConstants;
import frc.robot.subsystems.lidarSensor.LidarSensorSubsystemConstants;
import frc.robot.subsystems.pneumatics.PneumaticsSubsystemConstants;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants;
import frc.robot.subsystems.wrist.WristSubsystemConstants;
import frc.robot.triggermaps.*;

import static digilib.DigiMath.roundToDecimal;

public class Robit extends TimedRobot {

    public Robit() {
        SmartDashboard.putString("Robot Comments", Constants.robotComments);
        PortForwarder.add(5800, "orangepi50.local", 5800);
        CanBridge.runTCP();

        double deadband = 0.10;
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController operatorController = new CommandXboxController(1);
        CommandJoystick climberController = new CommandJoystick(2);
        CommandXboxController manualController = new CommandXboxController(3);

        SwerveDriveSubsystem swerveDriveSubsystem = SwerveDriveSubsystemConstants.createCTRESwerveDrive();
        AutoFactory autoFactory = SwerveDriveSubsystemConstants.getAutoFactory();

        Mechanism2d manipulatorMechanism = new Mechanism2d(3, 5, new Color8Bit(Color.kMediumPurple));
        MechanismRoot2d manipulatorRoot = manipulatorMechanism.getRoot("Manipulator", 1.5, 2);
        MechanismLigament2d elevatorLigament = new MechanismLigament2d("Elevator", 0.20, 90, 2, new Color8Bit(Color.kYellow));
        MechanismLigament2d bar = new MechanismLigament2d("ElevatorBackBar", 1, 90, 2, new Color8Bit(Color.kYellow));
        MechanismLigament2d armLigament = new MechanismLigament2d("Arm", 0.60, -90, 2, new Color8Bit(Color.kAntiqueWhite));
        MechanismLigament2d wristTopLigament = new MechanismLigament2d("WristTop", 0.3, 90, 2, new Color8Bit(Color.kDarkOrange));
        MechanismLigament2d wristBottomLigament = new MechanismLigament2d("WristBot", 0.3, -90, 2, new Color8Bit(Color.kGreen));
        manipulatorRoot.append(elevatorLigament);
        manipulatorRoot.append(bar);
        elevatorLigament.append(armLigament);
        armLigament.append(wristTopLigament);
        armLigament.append(wristBottomLigament);

        SmartDashboard.putData("Manipulator", manipulatorMechanism);

        Manipulator manipulator = new Manipulator(
                ArmSubsystemConstants.create(armLigament, -90),
                PneumaticsSubsystemConstants.createAlgaeClaw(),
                PneumaticsSubsystemConstants.createCoralClaw(),
                ElevatorSubsystemConstants.create(elevatorLigament, 0.20),
                AlgaeIntakeSubsystemConstants.create(),
                CoralIntakeSubsystemConstants.create(),
                LidarSensorSubsystemConstants.create(),
                WristSubsystemConstants.create(wristTopLigament, wristBottomLigament));
        ClimberSubsystem climberSubsystem = ClimberSubsystemConstants.create();

        AlgaePickup algaePickup = new AlgaePickup(manipulator);
        AlgaeScore algaeScore = new AlgaeScore(manipulator);
        CoralPickup coralPickup = new CoralPickup(manipulator);
        CoralScore coralScore = new CoralScore(manipulator);
        Manual manual = new Manual(swerveDriveSubsystem, manipulator);

        if (RobotBase.isSimulation()) {
            Timer timer = new Timer();
            addPeriodic(() -> {
                if (!timer.isRunning() && timer.get() <= 15.0 && RobotModeTriggers.autonomous().getAsBoolean()) {
                    timer.start();
                }
                if (RobotModeTriggers.autonomous().getAsBoolean()) {
                    SmartDashboard.putNumber("Auto Timer", roundToDecimal(15 - timer.get(), 2));
                } else if (RobotModeTriggers.teleop().getAsBoolean() || timer.get() > 15.0) {
                    timer.stop();
                    timer.reset();
                } else if (RobotModeTriggers.disabled().getAsBoolean()) {
                    timer.stop();
                    timer.reset();
                }
            }, 0.020);
        }

        new DriveMap(driverController, deadband, swerveDriveSubsystem, autoFactory);
        new AlgaePickupMap(operatorController, deadband, algaePickup);
        new AlgaeScoreMap(driverController, operatorController, deadband, algaeScore);
        new CoralPickupMap(operatorController, coralPickup);
        new CoralScoreMap(driverController, operatorController, coralScore);
        new ClimberMap(climberController, deadband, climberSubsystem);
        new ManualMap(manualController, operatorController, deadband, manual);
        new AutoRoutines(autoFactory, coralPickup, coralScore);
        SmartDashboard.putData(CommandScheduler.getInstance());
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);
        addPeriodic(CommandScheduler.getInstance()::run, 0.020);
    }
}
