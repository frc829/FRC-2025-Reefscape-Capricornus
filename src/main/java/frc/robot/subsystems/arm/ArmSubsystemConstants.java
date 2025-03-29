package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import digilib.arm.Arm;
import digilib.arm.TalonFXArm;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.FusedCANcoder;
import static com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static digilib.arm.Arm.*;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.rio;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.AbsoluteEncoder.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Control.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Mechanism.reduction;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Motor.talonFX;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Simulation.simLoopPeriod;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Simulation.startingAngleDegrees;

public class ArmSubsystemConstants {

    static final class Control {
        static final double ksVolts = 0.36397;
        static final double kgVolts = 0.082338;
        static final double kvVoltsPerRPS = 37.34;
        static final double kaVoltsPerRPSSquared = 1.5197;
        static final GravityTypeValue gravityTypeValue = Arm_Cosine;
        static final double positionKpVoltsPerRotation = 100.0; //197.69;
        static final double positionKdVoltsPerRPS = 0.0; // 12.045;
        static final double velocityKpVoltsPerRPS = 1.699E-15;
        static final double maxControlVoltage = 12.0;
        static final double maxVelocityRPS = maxControlVoltage / kvVoltsPerRPS;
        static final double maxAccelerationRPSS = maxControlVoltage / kaVoltsPerRPSSquared;
    }

    static final class Simulation {
        static final double startingAngleDegrees = 0.0;
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    static final class Mechanism {
        static final String name = "Arm";
        static final double reduction = 5.0 * 5.0 * 4.0 * 72.0 / 22.0;
        static final double minAngleDegrees = -50.0;
        static final double maxAngleDegrees = 180.0;
        static final Config config = new Config(
                name,
                reduction,
                startingAngleDegrees,
                minAngleDegrees,
                maxAngleDegrees,
                ksVolts,
                kgVolts,
                kvVoltsPerRPS,
                kaVoltsPerRPSSquared,
                maxVelocityRPS,
                maxAccelerationRPSS);
    }

    static final class AbsoluteEncoder {
        static final int cancoderDeviceNumber = 34;
        static final double magnetDirection = RobotBase.isReal()
                ? 0.412842
                : 0.0;
        static final FeedbackSensorSourceValue feedbackSensorSourceValue = FusedCANcoder;
        static final SensorDirectionValue sensorDirectionValue = RobotBase.isReal()
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        static final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
                .withSensorDirection(sensorDirectionValue)
                .withMagnetOffset(magnetDirection);
        static final CANcoderConfiguration config = new CANcoderConfiguration()
                .withMagnetSensor(magnetSensorConfigs);
        static final CANcoder cancoder = new CANcoder(cancoderDeviceNumber, rio);
    }

    static final class Motor {
        static final int deviceNumber = 14;
        static final NeutralModeValue neutralModeValue = Brake;
        static final InvertedValue invertedValue = CounterClockwise_Positive;
        static final double supplyCurrentLimitAmps = 70;
        static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(cancoderDeviceNumber)
                .withFeedbackSensorSource(feedbackSensorSourceValue)
                .withSensorToMechanismRatio(1.0)
                .withRotorToSensorRatio(reduction);
        static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicExpo_kV(kvVoltsPerRPS)
                .withMotionMagicExpo_kA(kaVoltsPerRPSSquared)
                .withMotionMagicAcceleration(maxAccelerationRPSS);
        static final VoltageConfigs voltageConfigs = new VoltageConfigs()
                .withPeakForwardVoltage(maxControlVoltage)
                .withPeakReverseVoltage(-maxControlVoltage);
        static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(supplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true);
        static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(invertedValue)
                .withNeutralMode(neutralModeValue);
        static final Slot0Configs slot0Configs = new Slot0Configs()
                .withKS(ksVolts)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKG(kgVolts)
                .withKV(kvVoltsPerRPS)
                .withKA(kaVoltsPerRPSSquared)
                .withGravityType(gravityTypeValue)
                .withKP(positionKpVoltsPerRotation)
                .withKD(positionKdVoltsPerRPS);
        static final Slot1Configs slot1Configs = new Slot1Configs()
                .withKS(ksVolts)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKG(kgVolts)
                .withKV(kvVoltsPerRPS)
                .withKA(kaVoltsPerRPSSquared)
                .withGravityType(gravityTypeValue)
                .withKP(velocityKpVoltsPerRPS);
        static final TalonFXConfiguration config = new TalonFXConfiguration()
                .withCurrentLimits(currentLimitsConfigs)
                .withVoltage(voltageConfigs)
                .withFeedback(feedbackConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withSlot0(slot0Configs)
                .withSlot1(slot1Configs);
        static final TalonFX talonFX = new TalonFX(deviceNumber, rio);
    }

    public static ArmSubsystem create(MechanismLigament2d ligament, double offsetDegrees) {
        cancoder.getConfigurator().apply(AbsoluteEncoder.config);
        talonFX.getConfigurator().apply(Motor.config);
        Arm arm = new TalonFXArm(
                Mechanism.config,
                talonFX,
                cancoder,
                ligament,
                offsetDegrees);
        ArmSubsystem armSubsystem = new ArmSubsystem(arm, simLoopPeriod);
        armSubsystem.setDefaultCommand(armSubsystem.hold());
        return armSubsystem;
    }
}
