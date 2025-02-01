package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.mechanisms.arm.Arm;
import frc.robot.mechanisms.arm.KrakenX60Arm;
import frc.robot.subsystems.CommandArm;

public class ArmConstants {
    private static final Angle minAngle = Degrees.of(-20);
    private static final Angle maxAngle = Degrees.of(200);
    private static final Distance armLength = Meters.of(1.0);


    private static final int deviceNumber = 14;
    private static final NeutralModeValue neutralModeValue = NeutralModeValue.Brake;
    private static final InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
    private static final double reduction = 1.0; // TODO: figure this out
    private static final Voltage ks = Volts.of(0.0);
    private static final Voltage kg = Volts.of(0.0);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of(1.0);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of(1.0);
    private static final double positionKp = 0.0;
    private static final double positionKd = 0.0;
    private static final double velocityKp = 0.0;
    private static final TalonFXConfiguration config = new TalonFXConfiguration();


    private static final int canCoderDeviceNumber = 34;
    private static final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    private static final double magnetDirection = 0.0;

    public static CommandArm createCommandArm() {
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = magnetDirection;
        CANcoder caNcoder = new CANcoder(canCoderDeviceNumber, UniversalRobotConstants.rio);
        caNcoder.getConfigurator().apply(cancoderConfig);

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        config.Feedback.FeedbackRemoteSensorID = canCoderDeviceNumber;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        config.Feedback.RotorToSensorRatio = 1.0;
        config.Feedback.SensorToMechanismRatio = reduction;
        config.MotionMagic.MotionMagicExpo_kV = kv.baseUnitMagnitude();
        config.MotionMagic.MotionMagicExpo_kA = ka.baseUnitMagnitude();
        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = neutralModeValue;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kP = positionKp;
        config.Slot0.kD = positionKd;
        config.Slot0.kS = ks.baseUnitMagnitude();
        config.Slot0.kV = kv.baseUnitMagnitude();
        config.Slot0.kA = ka.baseUnitMagnitude();
        config.Slot0.kG = kg.baseUnitMagnitude();
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        config.Slot1.kP = velocityKp;
        config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot1.kS = ks.baseUnitMagnitude();
        config.Slot1.kV = kv.baseUnitMagnitude();
        config.Slot1.kA = ka.baseUnitMagnitude();
        config.Slot1.kG = kg.baseUnitMagnitude();
        config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        TalonFX talonFX = new TalonFX(deviceNumber, UniversalRobotConstants.rio);
        talonFX.getConfigurator().apply(config);

        frc.robot.mechanisms.arm.ArmConstants armConstants = new frc.robot.mechanisms.arm.ArmConstants(
            maxAngle, 
            minAngle, 
            ks, 
            kg, 
            kv, 
            ka, 
            armLength, 
            reduction, 
            Radians.of(0.0),
            Radians.of(0.0), 
            RadiansPerSecond.of(0.0));
        
        Arm arm = new KrakenX60Arm(armConstants, talonFX, caNcoder);

        return null;

    }


}
