package frc.robot.subsystems.winch;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import digilib.winch.KrakenX60Winch;
import digilib.winch.Winch;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

import static com.ctre.phoenix6.signals.NeutralModeValue.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

public class CommandWinchConstants {
    private static final int deviceNumber = 18;
    private static final NeutralModeValue neutralModeValue = Brake;
    private static final InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
    private static final Distance drumRadius = Meters.of(1.0);
    private static final double reduction = 1.0;
    private static final Time simLoopPeriod = Seconds.of(0.001);

    public static CommandWinch createCommandWinch() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Feedback.SensorToMechanismRatio = reduction;
        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = neutralModeValue;
        TalonFX motor = new TalonFX(deviceNumber, Constants.rio);
        motor.getConfigurator().apply(config);
        Winch winch = new KrakenX60Winch(motor, drumRadius);
        return new CommandWinch(winch, simLoopPeriod);
    }
}
