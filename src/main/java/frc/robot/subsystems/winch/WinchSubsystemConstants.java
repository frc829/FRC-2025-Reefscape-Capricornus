package frc.robot.subsystems.winch;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import digilib.winch.VortexWinch;
import digilib.winch.Winch;
import digilib.winch.WinchConstants;
import edu.wpi.first.units.measure.*;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.winch.WinchSubsystemConstants.Mechanism.constants;
import static frc.robot.subsystems.winch.WinchSubsystemConstants.Motor.config;
import static frc.robot.subsystems.winch.WinchSubsystemConstants.Motor.sparkFlex;
import static frc.robot.subsystems.winch.WinchSubsystemConstants.Simulation.simLoopPeriod;

public class WinchSubsystemConstants {

    static final class Mechanism {
        static final String name = "Wrist";
        static final WinchConstants constants = new WinchConstants(name);
    }

    static final class Motor {
        static final int deviceNumber = 18;
        private static final SparkBaseConfig.IdleMode mode = kBrake;
        private static final boolean inverted = false;
        static final SparkBaseConfig config = new SparkFlexConfig()
                .inverted(inverted)
                .idleMode(mode);
        static final SparkFlex sparkFlex = new SparkFlex(deviceNumber, kBrushless);
    }

    static final class Simulation {
        static final Time simLoopPeriod = Seconds.of(0.001);
    }


    public static WinchSubsystem create() {
        sparkFlex.configure(config, kResetSafeParameters, kPersistParameters);
        Winch winch = new VortexWinch(constants, sparkFlex);
        WinchSubsystem winchSubsystem = new WinchSubsystem(winch, simLoopPeriod);
        winchSubsystem.setDefaultCommand(winchSubsystem.idle());
        return winchSubsystem;
    }
}
