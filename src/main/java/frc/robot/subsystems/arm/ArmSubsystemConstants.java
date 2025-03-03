package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.AbsoluteEncoder.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Control.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Mechanism.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Motor.*;
import static frc.robot.subsystems.arm.ArmSubsystemConstants.Simulation.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import digilib.arm.Arm;
import digilib.arm.ArmConstants;
import digilib.arm.TalonFXArm;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class ArmSubsystemConstants {

    static final class Control {
        static final Voltage ks = Volts.of(0.14546);
        static final Voltage kg = Volts.of(0.13317);
        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RotationsPerSecond).of(39.295);
        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RotationsPerSecondPerSecond).of(0.33517);
        static final GravityTypeValue gravityTypeValue = GravityTypeValue.Arm_Cosine;
        static final double positionKp = 96.725;
        static final double positionKd = 0.2086;
        static final double velocityKp = 2.0133;
        static final AngularVelocity maxAngularVelocity = RotationsPerSecond.of(
                (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / kv.magnitude());
        static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(
                (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / ka.magnitude());
    }

    static final class Mechanism {
        static final String name = "Arm";
        static final double reduction = 5.0 * 5.0 * 4.0 * 72.0 / 22.0;
        static final ArmConstants constants = new ArmConstants(
                name,
                reduction,
                maxAngle,
                minAngle,
                startingAngle,
                ks,
                kg,
                kv,
                ka,
                maxAngularVelocity,
                maxAngularAcceleration,
                positionStdDev,
                velocityStdDev);
    }

    static final class Simulation {
        static final Angle startingAngle = Degrees.of(0.0);
        static final Angle minAngle = Degrees.of(-50.0);
        static final Angle maxAngle = Degrees.of(180.0);
        static final Angle positionStdDev = Degrees.of(0.0);
        static final AngularVelocity velocityStdDev = DegreesPerSecond.of(0.0);
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    static final class AbsoluteEncoder {
        static final int cancoderDeviceNumber = 34;
        static final double magnetDirection = RobotBase.isReal() ? 0.093506 : 0.0;  // 0.088623
        static final FeedbackSensorSourceValue feedbackSensorSourceValue = FeedbackSensorSourceValue.FusedCANcoder;
        static final SensorDirectionValue sensorDirectionValue = RobotBase.isReal()
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        static final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
                .withSensorDirection(sensorDirectionValue)
                .withMagnetOffset(magnetDirection);
        static final CANcoderConfiguration config = new CANcoderConfiguration()
                .withMagnetSensor(magnetSensorConfigs);
        static final CANcoder cancoder = new CANcoder(cancoderDeviceNumber, Constants.rio);
    }

    static final class Motor {
        static final int deviceNumber = 14;
        static final InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
        static final NeutralModeValue neutralModeValue = NeutralModeValue.Brake;
        static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(cancoderDeviceNumber)
                .withFeedbackSensorSource(feedbackSensorSourceValue)
                .withSensorToMechanismRatio(1.0)
                .withRotorToSensorRatio(reduction);
        static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicExpo_kV(kv.magnitude())
                .withMotionMagicExpo_kA(ka.magnitude())
                .withMotionMagicCruiseVelocity(maxAngularVelocity)
                .withMotionMagicAcceleration(maxAngularAcceleration);
        static final VoltageConfigs voltageConfigs = new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0);
        static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(invertedValue)
                .withNeutralMode(neutralModeValue);
        static final Slot0Configs slot0Configs = new Slot0Configs()
                .withKS(ks.baseUnitMagnitude())
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKG(kg.baseUnitMagnitude())
                .withKV(kv.magnitude())
                .withKA(ka.magnitude())
                .withGravityType(gravityTypeValue)
                .withKP(positionKp)
                .withKD(positionKd);
        static final Slot1Configs slot1Configs = new Slot1Configs()
                .withKS(ks.baseUnitMagnitude())
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKG(kg.baseUnitMagnitude())
                .withKV(kv.magnitude())
                .withKA(ka.magnitude())
                .withGravityType(gravityTypeValue)
                .withKP(velocityKp);
        static final TalonFX talonFX = new TalonFX(deviceNumber, Constants.rio);
        static final TalonFXConfiguration config = new TalonFXConfiguration()
                .withVoltage(voltageConfigs)
                .withFeedback(feedbackConfigs)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withSlot0(slot0Configs)
                .withSlot1(slot1Configs);
    }

    public static ArmSubsystem create() {
        cancoder.getConfigurator().apply(AbsoluteEncoder.config);
        talonFX.getConfigurator().apply(Motor.config);
        Arm arm = new TalonFXArm(constants, talonFX, cancoder);
        ArmSubsystem armSubsystem = new ArmSubsystem(arm, simLoopPeriod);
        armSubsystem.setDefaultCommand(armSubsystem.hold());
        return armSubsystem;
    }
}
