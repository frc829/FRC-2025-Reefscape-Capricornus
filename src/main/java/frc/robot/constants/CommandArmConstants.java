package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.mechanisms.arm.Arm;
import frc.robot.mechanisms.arm.ArmConstants;
import frc.robot.mechanisms.arm.KrakenX60Arm;
import frc.robot.subsystems.CommandArm;

public class CommandArmConstants {
    private static final Angle minAngle = Degrees.of(-20);
    private static final Angle maxAngle = Degrees.of(20);
    private static final Distance armLength = Meters.of(1.0);
    private static final int deviceNumber = 14;
    private static final NeutralModeValue neutralModeValue = NeutralModeValue.Brake;
    private static final InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
    private static final double reduction = 1.0;
    private static final Voltage ks = Volts.of(0.0);
    private static final Voltage kg = Volts.of(0.0);

    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of(1.0);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of(1.0);

    private static final double positionKp = 0.0;
    private static final double positionKd = 0.0;
    private static final double velocityKp = 0.0;

    private static final int cancoderDeviceNumber = 34;
    private static final double magnetDirection = 0.0;
    private static final Time simLoopPeriod = Seconds.of(0.001);



    public static CommandArm createCommandArm() {
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = magnetDirection;
        CANcoder cancoder =  new CANcoder(cancoderDeviceNumber, UniversalRobotConstants.rio);
        cancoder.getConfigurator().apply(cancoderConfig);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Voltage.PeakForwardVoltage = 12.0;
        talonFXConfiguration.Voltage.PeakReverseVoltage = -12.0;
        talonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = cancoderDeviceNumber;
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        talonFXConfiguration.Feedback.RotorToSensorRatio = reduction;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 1.0;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = kv.baseUnitMagnitude();
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = ka.baseUnitMagnitude();
        talonFXConfiguration.MotorOutput.Inverted = invertedValue;
        talonFXConfiguration.MotorOutput.NeutralMode = neutralModeValue;
        talonFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        talonFXConfiguration.Slot0.kP = positionKp;
        talonFXConfiguration.Slot0.kD = positionKd;
        talonFXConfiguration.Slot0.kS= ks.baseUnitMagnitude();
        talonFXConfiguration.Slot0.kV = kv.baseUnitMagnitude();
        talonFXConfiguration.Slot0.kA = ka.baseUnitMagnitude();
        talonFXConfiguration.Slot0.kG = kg.baseUnitMagnitude();
        talonFXConfiguration.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        talonFXConfiguration.Slot1.kP = velocityKp;
        talonFXConfiguration.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        talonFXConfiguration.Slot1.kS = ks.baseUnitMagnitude();
        talonFXConfiguration.Slot1.kV = kv.baseUnitMagnitude();
        talonFXConfiguration.Slot1.kA = ka.baseUnitMagnitude();
        talonFXConfiguration.Slot1.kG = kg.baseUnitMagnitude();
        talonFXConfiguration.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        TalonFX talonFX = new TalonFX(deviceNumber, UniversalRobotConstants.rio);
        talonFX.getConfigurator().apply(talonFXConfiguration);


        ArmConstants armConstants = new ArmConstants(maxAngle, minAngle, ks, kg, kv, ka, armLength, reduction, Radians.of(0.0), Radians.of(0.0), RadiansPerSecond.of(0.0) );


        Arm arm = new KrakenX60Arm(armConstants, talonFX, cancoder);
        return new CommandArm(arm, simLoopPeriod);


    }


}
