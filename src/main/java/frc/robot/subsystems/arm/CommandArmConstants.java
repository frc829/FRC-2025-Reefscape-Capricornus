package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import digilib.arm.Arm;
import digilib.arm.ArmConstants;
import digilib.arm.KrakenX60Arm;
import frc.robot.Constants;

public class CommandArmConstants {
    private static final String name = "Arm";
    private static final Angle startingAngle = Degrees.of(-45);
    private static final Angle minAngle = Degrees.of(-45);
    private static final Angle maxAngle = Degrees.of(100);
    private static final Distance armLength = Inches.of(31.0);
    private static final int deviceNumber = 14;
    private static final NeutralModeValue neutralModeValue = NeutralModeValue.Brake;
    private static final InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
    private static final double reduction = 5.0 * 5.0 * 4.0 * 72.0 / 22.0;
    private static final Voltage ks = Volts.of(0.19744);
    private static final Voltage kg = Volts.of(0.15216);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RotationsPerSecond).of(38.776);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RotationsPerSecondPerSecond).of(0.35225);
    private static final double positionKp = 100.84;
    private static final double positionKd = 0.0;
    private static final double velocityKp = 19.725;
    private static final int cancoderDeviceNumber = 34;
    private static final double magnetDirection = 0.0;
    private static final Time simLoopPeriod = Seconds.of(0.001);
    private static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(
            (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / kv.baseUnitMagnitude());
    private static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(
            (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / ka.baseUnitMagnitude());


    public static CommandArm createCommandArm() {
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = magnetDirection;
        CANcoder cancoder = new CANcoder(cancoderDeviceNumber, Constants.rio);
        cancoder.getConfigurator().apply(cancoderConfig);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Voltage.PeakForwardVoltage = 12.0;
        talonFXConfiguration.Voltage.PeakReverseVoltage = -12.0;
        talonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = false;
        if (RobotBase.isReal()) {
            talonFXConfiguration.Feedback.FeedbackRemoteSensorID = cancoderDeviceNumber;
            talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        }
        talonFXConfiguration.Feedback.RotorToSensorRatio = 1.0;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = reduction;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = kv.baseUnitMagnitude();
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = ka.baseUnitMagnitude();
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = maxAngularVelocity.in(RotationsPerSecond);
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = maxAngularAcceleration.in(RotationsPerSecondPerSecond);
        talonFXConfiguration.MotorOutput.Inverted = invertedValue;
        talonFXConfiguration.MotorOutput.NeutralMode = neutralModeValue;
        talonFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        talonFXConfiguration.Slot0.kP = positionKp;
        talonFXConfiguration.Slot0.kD = positionKd;
        talonFXConfiguration.Slot0.kS = ks.baseUnitMagnitude();
        talonFXConfiguration.Slot0.kV = kv.baseUnitMagnitude();
        talonFXConfiguration.Slot0.kA = ka.baseUnitMagnitude();
        talonFXConfiguration.Slot0.kG = kg.baseUnitMagnitude();
        talonFXConfiguration.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        talonFXConfiguration.Slot1.kP = velocityKp;
        talonFXConfiguration.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        talonFXConfiguration.Slot1.kS = ks.baseUnitMagnitude();
        talonFXConfiguration.Slot1.kV = kv.baseUnitMagnitude() * 2 * Math.PI;
        talonFXConfiguration.Slot1.kA = ka.baseUnitMagnitude() * 2 * Math.PI;
        talonFXConfiguration.Slot1.kG = kg.baseUnitMagnitude();
        talonFXConfiguration.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        TalonFX talonFX = new TalonFX(deviceNumber, Constants.rio);
        talonFX.getConfigurator().apply(talonFXConfiguration);


        ArmConstants armConstants = new ArmConstants(
                name,
                maxAngle, minAngle, maxAngularVelocity, maxAngularAcceleration, ks, kg, kv, ka, armLength, reduction, Radians.of(0.0), Radians.of(0.0), RadiansPerSecond.of(0.0));


        Arm arm = new KrakenX60Arm(armConstants, talonFX, cancoder);
        return new CommandArm(arm, simLoopPeriod);


    }


}
