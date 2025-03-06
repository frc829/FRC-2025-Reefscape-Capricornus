package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import digilib.wrist.NEO550Wrist;
import digilib.wrist.Wrist;
import digilib.wrist.WristConstants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.AbsoluteEncoder.cancoder;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Control.*;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Mechanism.*;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Mechanism.reduction;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Motor.*;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Simulation.*;

public class WristSubsystemConstants {

    static final class Control {
        static final double ksVolts = 0.13474;
        static final double kvVoltsPerRPS = 0.81628;
        static final double kaVoltsPerRPSSquared = 0.028768;
        static final double positionKpVoltsPerRotation = 0.0;
        static final double positionKdVoltsPerRPS = 0.0;
        static final double velocityKpVoltsPerRPS = 0.0;
        static final double maxControlVoltage = 12.0 - ksVolts;
        static final double maxVelocityRPS = maxControlVoltage / kvVoltsPerRPS;
        static final double maxAccelerationRPSS = maxControlVoltage / kaVoltsPerRPSSquared;
        static final double controlPeriodSeconds = 0.020;
    }

    static final class Simulation {
        static final double startingAngleDegrees = 0.0;
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    static final class Mechanism {
        static final String name = "Wrist";
        static final double reduction = 4.0 * 3.0 * 5.0 * 32.0 / 24.0;   // 10 7 32.0 / 18
        static final double minAngleDegrees = -10.0;
        static final double maxAngleDegrees = 100.0;
        static final WristConstants constants = new WristConstants(
                name,
                reduction,
                startingAngleDegrees,
                minAngleDegrees,
                maxAngleDegrees,
                maxControlVoltage,
                ksVolts,
                kvVoltsPerRPS,
                kaVoltsPerRPSSquared,
                maxVelocityRPS,
                maxAccelerationRPSS);
    }

    static final class AbsoluteEncoder {
        static final int cancoderDeviceNumber = 37;
        static final SensorDirectionValue sensorDirectionValue = RobotBase.isReal()
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        static final double magnetOffset = RobotBase.isReal()
                ? 0.439453
                : 0.0;
        static final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
                .withSensorDirection(sensorDirectionValue)
                .withMagnetOffset(magnetOffset);
        static final CANcoderConfiguration config = new CANcoderConfiguration()
                .withMagnetSensor(magnetSensorConfigs);
        static final CANcoder cancoder = new CANcoder(cancoderDeviceNumber, rio);
    }

    static final class Motor {
        static final int deviceId = 17;
        static final SparkBaseConfig.IdleMode idleMode = kBrake;
        static final boolean inverted = true;
        static final int smartCurrentLimit = 20;
        static final int depth = 2;
        static final int periodMs = 16;
        static final double positionFactor = reduction;
        static final double velocityFactor = reduction / 60.0;
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
                .apply(encoderConfig)
                .smartCurrentLimit(20);
        static final SparkMax motor = new SparkMax(deviceId, kBrushless);
        static final PIDController positionPIDController = new PIDController(positionKpVoltsPerRotation, 0.0, positionKdVoltsPerRPS, controlPeriodSeconds);
        static final PIDController velocityPIDController = new PIDController(velocityKpVoltsPerRPS, 0.0, 0.0, controlPeriodSeconds);
    }

    public static WristSubsystem create() {
        cancoder.getConfigurator().apply(AbsoluteEncoder.config);
        motor.configure(Motor.config, kResetSafeParameters, kPersistParameters);
        Wrist wrist = new NEO550Wrist(
                constants,
                motor,
                cancoder,
                positionPIDController,
                velocityPIDController,
                controlPeriodSeconds);
        WristSubsystem wristSubsystem = new WristSubsystem(wrist, simLoopPeriod);
        MutAngle holdPosition = Degrees.mutable(0.0);
        Command hold = runOnce(() -> holdPosition.mut_setBaseUnitMagnitude(wrist.getState().getAbsoluteEncoderPositionDegrees()))
                .andThen(wristSubsystem.toAngle(holdPosition.in(Degrees)));
        wristSubsystem.setDefaultCommand(hold);
        return wristSubsystem;
    }
}
