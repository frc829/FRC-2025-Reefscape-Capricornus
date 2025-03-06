package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import digilib.climber.VortexClimber;
import digilib.climber.Climber;
import digilib.climber.ClimberConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberSubsystemConstants.Control.*;
import static frc.robot.subsystems.climber.ClimberSubsystemConstants.Mechanism.*;
import static frc.robot.subsystems.climber.ClimberSubsystemConstants.Motor.*;
import static frc.robot.subsystems.climber.ClimberSubsystemConstants.Simulation.simLoopPeriod;
import static frc.robot.subsystems.climber.ClimberSubsystemConstants.Simulation.startingLengthMeters;

public class ClimberSubsystemConstants {

    static final class Control {
        static final DCMotor dcMotor = DCMotor.getNeoVortex(1);
        static final double motorMaxSpeedRadPerSec = dcMotor.freeSpeedRadPerSec;
        static final double climberMaxSpeedMPS = motorMaxSpeedRadPerSec * drumRadiusMeters / reduction;
        static final double ksVolts = 0.0;
        static final double maxControlVoltage = 12.0 - ksVolts;
        static final double climberMassKilograms = 1.0;
        static final double climberMaxAccelerationMPSS = dcMotor.rOhms * drumRadiusMeters * climberMassKilograms / reduction / dcMotor.KtNMPerAmp;
        static final double kvVoltsPerMPS = maxControlVoltage / climberMaxSpeedMPS;
        static final double kaVoltsPerMPSSquared = maxControlVoltage / climberMaxAccelerationMPSS;
        static final double maxVelocityMPS = maxControlVoltage / kvVoltsPerMPS;
        static final double maxAccelerationMPSS = maxControlVoltage / kaVoltsPerMPSSquared;
    }

    static final class Simulation {
        static final double startingLengthMeters = 0.0;
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    static final class Mechanism {
        static final String name = "Climber";
        static final double reduction = 9 * 5 * 4;
        static final double sprocketPitchDiameterInches = 2;
        static final double sprocketPitchRadiusInches = sprocketPitchDiameterInches / 2;
        static final double drumRadiusMeters = Units.inchesToMeters(sprocketPitchRadiusInches);
        static final double minLengthCentimeters = 0.0;
        static final double minLengthMeters = minLengthCentimeters / 100.0;
        static final double maxLengthCentimeters = 20;
        static final double maxLengthMeters = maxLengthCentimeters / 100.0;
        static final ClimberConstants constants = new ClimberConstants(
                name,
                reduction,
                drumRadiusMeters,
                startingLengthMeters,
                minLengthMeters,
                maxLengthMeters,
                maxControlVoltage,
                ksVolts,
                kvVoltsPerMPS,
                kaVoltsPerMPSSquared,
                maxVelocityMPS,
                maxAccelerationMPSS);
    }

    static final class Motor {
        static final int deviceNumber = 18;
        private static final IdleMode idleMode = kBrake;
        private static final boolean inverted = false;
        static final int smartCurrentLimit = 80;
        static final int depth = 2;
        static final int periodMs = 16;
        static final double positionFactor = 2 * Math.PI * drumRadiusMeters / reduction;
        static final double velocityFactor = 2 * Math.PI * drumRadiusMeters / reduction / 60.0;
        static final SignalsConfig signalsConfig = new SignalsConfig()
                .absoluteEncoderPositionAlwaysOn(false)
                .absoluteEncoderVelocityAlwaysOn(false)
                .analogPositionAlwaysOn(false)
                .analogVelocityAlwaysOn(false)
                .externalOrAltEncoderPositionAlwaysOn(false)
                .externalOrAltEncoderVelocityAlwaysOn(false)
                .primaryEncoderPositionAlwaysOn(false)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20);
        static final EncoderConfig encoderConfig = new EncoderConfig()
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(velocityFactor)
                .quadratureAverageDepth(depth)
                .quadratureMeasurementPeriod(periodMs);
        static final SparkBaseConfig config = new SparkFlexConfig()
                .idleMode(idleMode)
                .inverted(inverted)
                .smartCurrentLimit(smartCurrentLimit)
                .apply(signalsConfig)
                .apply(encoderConfig);
        static final SparkFlex motor = new SparkFlex(deviceNumber, kBrushless);
    }

    public static ClimberSubsystem create() {
        motor.configure(config, kResetSafeParameters, kPersistParameters);
        Climber climber = new VortexClimber(constants, motor);
        ClimberSubsystem climberSubsystem = new ClimberSubsystem(climber, simLoopPeriod);
        climberSubsystem.setDefaultCommand(climberSubsystem.toVoltage(0.0));
        return climberSubsystem;
    }
}
