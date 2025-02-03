package frc.robot.constants;

import frc.robot.subsystems.CommandWinch;

public class CommandWinchConstants {
    // TODO: all of these are private static final fields
    // TODO: int deviceNumber assign 18 to it.
    // TODO: NeutralModeValue named neutralModeValue assign NeutralModelValue.Brake to it
    // TODO: InvertedValue named invertedValue and assign InvertedValue.CounterClockwise_Positive to it
    // TODO: Distance named drumRadius and assign Meters.of(1.0) to it (for now)
    // TODO: double named reduction assign 1.0 to it.
    // TODO: Time named simLoopPeriod and assign Seconds.of(0.001) to it.


    public static CommandWinch createCommandWinch() {
        // TODO: create a TalonFXConfiguration named config and assign new TalonFXConfiguration() to it.
        // TODO: assign 12.0 to config.Voltage.PeakForwardVoltage
        // TODO: repeat on PeakReverseVoltage with -12.0;
        // TODO: assign reduction to config.Feedback.SensorToMechanismRatio
        // TODO: assign invertedValue to config.MotorOutput.Inverted
        // TODO: assign neutralModeValue to config.MotorOutput.NeutralMode
        // TODO: create a TalonFX named motor and assign new TalonFX() passing in deviceNumber, and RobotConstants.rio
        // TODO: call motor.getConfigurator().apply() passing in config
        // TODO: create a new Winch named winch and assign new KrakenX60Winch() passing in motor and drumRadius to it.
        // TODO: return new CommandWinch() passing winch, and SimLoopPeriod to it.
        return null; // TODO: remove this when done.
    }
}
