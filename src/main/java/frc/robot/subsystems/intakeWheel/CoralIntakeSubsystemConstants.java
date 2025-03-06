package frc.robot.subsystems.intakeWheel;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.*;
import digilib.intakeWheel.IntakeWheel;
import digilib.intakeWheel.IntakeWheelConstants;
import digilib.intakeWheel.NEO550IntakeWheel;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intakeWheel.CoralIntakeSubsystemConstants.Control.*;
import static frc.robot.subsystems.intakeWheel.CoralIntakeSubsystemConstants.Mechanism.*;
import static frc.robot.subsystems.intakeWheel.CoralIntakeSubsystemConstants.Motor.*;
import static frc.robot.subsystems.intakeWheel.CoralIntakeSubsystemConstants.Simulation.*;

public class CoralIntakeSubsystemConstants {

    static final class Control {
        static final double ksVolts = 0.12198;
        static final double kvVoltsPerRPS = 0.12267 * 2 * Math.PI;
        static final double kaVoltsPerRPSSquared = 0.0045787 * 2 * Math.PI;
        static final double kpVoltsPerRPS = 0.00011686 * 2 * Math.PI;
        static final double maxControlVoltage = 12.0 - ksVolts;
        static final double maxVelocityRPS = maxControlVoltage / kvVoltsPerRPS;
        static final double maxAccelerationRPSS = maxControlVoltage / kaVoltsPerRPSSquared;
        static final double controlPeriodSeconds = 0.020;
    }

    static final class Mechanism {
        static final String name = "Intake: Coral";
        static final double reduction = 12.0;
        static final IntakeWheelConstants constants = new IntakeWheelConstants(
                name,
                maxControlVoltage,
                reduction,
                ksVolts,
                kvVoltsPerRPS,
                kaVoltsPerRPSSquared,
                maxVelocityRPS,
                maxAccelerationRPSS);
    }

    static final class Simulation {
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    static final class Motor {
        static final int deviceId = 26;
        static final IdleMode idleMode = kBrake;
        static final boolean inverted = false;
        static final int smartCurrentLimit = 20;
        static final int depth = 2;
        static final int periodMs = 16;
        static final double positionFactor = AlgaeIntakeSubsystemConstants.Mechanism.reduction;
        static final double velocityFactor = AlgaeIntakeSubsystemConstants.Mechanism.reduction / 60.0;
        static final SignalsConfig signalsConfig = new SignalsConfig()
                .absoluteEncoderPositionAlwaysOn(false)
                .absoluteEncoderVelocityAlwaysOn(false)
                .analogPositionAlwaysOn(false)
                .analogVelocityAlwaysOn(false)
                .externalOrAltEncoderPositionAlwaysOn(false)
                .externalOrAltEncoderVelocityAlwaysOn(false)
                .primaryEncoderPositionAlwaysOn(false)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20);
        static final EncoderConfig encoderConfig = new EncoderConfig()
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(velocityFactor)
                .uvwAverageDepth(depth)
                .uvwMeasurementPeriod(periodMs);
        static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
                .p(kpVoltsPerRPS, ClosedLoopSlot.kSlot1)
                .feedbackSensor(kPrimaryEncoder);
        static final SparkBaseConfig config = new SparkFlexConfig()
                .idleMode(idleMode)
                .inverted(inverted)
                .smartCurrentLimit(smartCurrentLimit)
                .apply(signalsConfig)
                .apply(encoderConfig)
                .apply(closedLoopConfig);
        static final SparkMax motor = new SparkMax(deviceId, kBrushless);
    }

    public static IntakeWheelSubsystem create() {
        motor.configure(config, kResetSafeParameters, kPersistParameters);
        IntakeWheel intakeWheel = new NEO550IntakeWheel(constants, motor, controlPeriodSeconds);
        IntakeWheelSubsystem intakeWheelSubsystem = new IntakeWheelSubsystem(intakeWheel, simLoopPeriod);
        intakeWheelSubsystem.setDefaultCommand(intakeWheelSubsystem.toVoltage(-0.75));
        return intakeWheelSubsystem;
    }
}
