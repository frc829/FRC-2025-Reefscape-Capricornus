package frc.robot.subsystems.winch;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import digilib.winch.KrakenX60Winch;
import digilib.winch.Winch;
import digilib.winch.WinchConstants;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

import static com.ctre.phoenix6.signals.NeutralModeValue.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.winch.WinchSubsystemConstants.Mechanism.constants;
import static frc.robot.subsystems.winch.WinchSubsystemConstants.Motor.config;
import static frc.robot.subsystems.winch.WinchSubsystemConstants.Motor.talonFX;
import static frc.robot.subsystems.winch.WinchSubsystemConstants.Simulation.simLoopPeriod;

public class WinchSubsystemConstants {

    static final class Mechanism {
        static final String name = "Wrist";
        static final WinchConstants constants = new WinchConstants(name);
    }

    static final class Motor {
        static final int deviceNumber = 18;
        private static final NeutralModeValue neutralModeValue = Brake;
        private static final InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
        static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(invertedValue)
                .withNeutralMode(neutralModeValue);
        static final TalonFX talonFX = new TalonFX(deviceNumber, Constants.rio);
        static final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs);
    }

    static final class Simulation {
        static final Time simLoopPeriod = Seconds.of(0.001);
    }


    public static WinchSubsystem create() {
        talonFX.getConfigurator().apply(config);
        Winch winch = new KrakenX60Winch(constants, talonFX);
        WinchSubsystem winchSubsystem = new WinchSubsystem(winch, simLoopPeriod);
        winchSubsystem.setDefaultCommand(winchSubsystem.idle());
        return winchSubsystem;
    }
}
