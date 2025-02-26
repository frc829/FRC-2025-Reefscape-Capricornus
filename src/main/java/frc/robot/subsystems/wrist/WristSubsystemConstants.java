package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import digilib.wrist.NEO550Wrist;
import digilib.wrist.Wrist;
import digilib.wrist.WristConstants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.AbsoluteEncoder.cancoder;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Control.*;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Mechanism.*;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Motor.closedLoopConfig;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Motor.motor;
import static frc.robot.subsystems.wrist.WristSubsystemConstants.Simulation.*;

public class WristSubsystemConstants {

    static final class Control {
        static final Voltage ks = Volts.of(0.12793);
        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of(1.0825);
        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of(0.27814);
        static final double positionKp = 9.8426;
        static final double positionKd = 0.66614;
        static final double velocityKp = 0.21318;
        static final Time updatePeriod = Seconds.of(0.020);
        static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(
                (12.0 - ks.baseUnitMagnitude()) / kv.baseUnitMagnitude());
        static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(
                (12.0 - ks.baseUnitMagnitude()) / ka.baseUnitMagnitude());
    }

    static final class Mechanism {
        static final String name = "Wrist";
        static final double reduction = 4.0 * 3.0 * 32.0 / 24.0;   // 10 7 32.0 / 18
        static final WristConstants constants = new WristConstants(
                name,
                reduction,
                maxAngle,
                minAngle,
                ks,
                kv,
                ka,
                maxAngularVelocity,
                maxAngularAcceleration,
                positionStdDev,
                velocityStdDev);
    }

    static final class Simulation {
        static final Angle minAngle = Degrees.of(0.0);
        static final Angle maxAngle = Degrees.of(90.0);
        static final Angle positionStdDev = Degrees.of(0.0);
        static final AngularVelocity velocityStdDev = DegreesPerSecond.of(0.0);
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    static final class AbsoluteEncoder {
        static final int cancoderDeviceNumber = 37;
        static final SensorDirectionValue sensorDirectionValue = RobotBase.isReal() ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        static final double magnetOffset = RobotBase.isReal()
                ? 0.421143
                : 0.0;
        static final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
                .withSensorDirection(sensorDirectionValue)
                .withMagnetOffset(magnetOffset);
        static final CANcoderConfiguration config = new CANcoderConfiguration()
                .withMagnetSensor(magnetSensorConfigs);
        static final CANcoder cancoder = new CANcoder(cancoderDeviceNumber, Constants.rio);
    }

    static final class Motor {
        static final int deviceId = 17;
        static final SparkBaseConfig.IdleMode idleMode = SparkBaseConfig.IdleMode.kBrake;
        static final boolean inverted = true;
        static final int depth = 2;
        static final int periodMs = 16;
        static final EncoderConfig encoderConfig = new EncoderConfig()
                .positionConversionFactor(2 * Math.PI / reduction)
                .velocityConversionFactor(2 * Math.PI / reduction / 60.0)
                .uvwAverageDepth(depth)
                .uvwMeasurementPeriod(periodMs);
        static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
                .pidf(positionKp, 0.0, positionKd, 0.0)
                .pidf(velocityKp, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1)
                .feedbackSensor(kPrimaryEncoder)
                .positionWrappingEnabled(false);
        static final SparkBaseConfig config = new SparkFlexConfig()
                .idleMode(idleMode)
                .inverted(inverted)
                .apply(encoderConfig)
                .apply(closedLoopConfig)
                .smartCurrentLimit(20);
        static final SparkMax motor = new SparkMax(deviceId, kBrushless);
    }

    public static WristSubsystem create() {
        cancoder.getConfigurator().apply(AbsoluteEncoder.config);
        motor.configure(Motor.config, kResetSafeParameters, kPersistParameters);
        Wrist wrist = new NEO550Wrist(
                constants,
                motor,
                cancoder,
                updatePeriod);
        WristSubsystem wristSubsystem = new WristSubsystem(wrist, simLoopPeriod);
        wristSubsystem.setDefaultCommand(wristSubsystem.hold());
        return wristSubsystem;
    }
}
