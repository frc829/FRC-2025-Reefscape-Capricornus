package frc.robot.subsystems.arm;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.*;
import static com.ctre.phoenix6.signals.GravityTypeValue.*;
import static com.ctre.phoenix6.signals.InvertedValue.*;
import static com.ctre.phoenix6.signals.NeutralModeValue.*;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.AbsoluteEncoder.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Control.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Mechanism.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Motor.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Simulation.*;


import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import digilib.arm.Arm;
import digilib.arm.ArmConstants;
import digilib.arm.TalonFXArm;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmSubsystemConstants {

    static final class Control {
        static final double ksVolts = 0.14546;
        static final double kgVolts = 0.13317;
        static final double kvVoltsPerRPS = 39.295;
        static final double kaVoltsPerRPSSquared = 0.33517;
        static final GravityTypeValue gravityTypeValue = Arm_Cosine;
        static final double positionKpVoltsPerRotation = 96.725;
        static final double positionKdVoltsPerRPS = 0.2086;
        static final double velocityKpVoltsPerRPS = 2.0133;
        static final double maxControlVoltage = 12.0 - ksVolts - kgVolts;
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
        static final ArmConstants constants = new ArmConstants(
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
                ? 0.093506
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

    public static ArmSubsystem create() {
        cancoder.getConfigurator().apply(AbsoluteEncoder.config);
        talonFX.getConfigurator().apply(Motor.config);
        Arm arm = new TalonFXArm(
                constants,
                talonFX,
                cancoder);
        ArmSubsystem armSubsystem = new ArmSubsystem(arm, simLoopPeriod);
        MutAngle holdPosition = Degrees.mutable(0.0);
        Command hold = runOnce(() -> holdPosition.mut_setBaseUnitMagnitude(arm.getState().getAbsoluteEncoderPositionDegrees()))
                .andThen(armSubsystem.toAngle(holdPosition.in(Degrees)));
        armSubsystem.setDefaultCommand(hold);
        return armSubsystem;
    }
}
