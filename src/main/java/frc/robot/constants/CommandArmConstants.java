package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.mechanisms.arm.Arm;
import frc.robot.mechanisms.arm.ArmConstants;
import frc.robot.mechanisms.arm.KrakenX60Arm;
import frc.robot.subsystems.CommandArm;

public class CommandArmConstants {
    private static final Angle minAngle = Degrees.of(-45);
    private static final Angle maxAngle = Degrees.of(225);
    private static final Distance armLength = Inches.of(31.0);
    private static final int deviceNumber = 14;
    private static final NeutralModeValue neutralModeValue = NeutralModeValue.Brake;
    private static final InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
    private static final double reduction = 5.0 * 5.0 * 4.0 * 72.0 / 22.0;
    private static final Voltage ks = Volts.of(0.0);

    private static final Mass armMass = Pounds.of(11.5);
    private static final MomentOfInertia momentOfInertia = KilogramSquareMeters.of(
            armMass.baseUnitMagnitude()
                    * Math.pow(armLength.baseUnitMagnitude(), 2)
                    / 3.0);

    private static final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);

    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of(reduction / dcMotor.KvRadPerSecPerVolt);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of(
            dcMotor.rOhms * momentOfInertia.baseUnitMagnitude() / dcMotor.KtNMPerAmp / reduction
    );

    private static final Voltage kg = Volts.of(
            3 * 9.8 * ka.baseUnitMagnitude() / 2 / armLength.baseUnitMagnitude()
    );


    private static final double positionKp = 20.0;
    private static final double positionKd = 0.0;
    private static final double velocityKp = 0.0;

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
        CANcoder cancoder = new CANcoder(cancoderDeviceNumber, RobotConstants.rio);
        cancoder.getConfigurator().apply(cancoderConfig);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Voltage.PeakForwardVoltage = 12.0;
        talonFXConfiguration.Voltage.PeakReverseVoltage = -12.0;
        talonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        if (RobotBase.isReal()) {
            talonFXConfiguration.Feedback.FeedbackRemoteSensorID = cancoderDeviceNumber;
            talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        }
        talonFXConfiguration.Feedback.RotorToSensorRatio = 1.0;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = reduction;
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kV = kv.baseUnitMagnitude();
        talonFXConfiguration.MotionMagic.MotionMagicExpo_kA = ka.baseUnitMagnitude();
        talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0.0;
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
        talonFXConfiguration.Slot1.kV = kv.baseUnitMagnitude();
        talonFXConfiguration.Slot1.kA = ka.baseUnitMagnitude();
        talonFXConfiguration.Slot1.kG = kg.baseUnitMagnitude();
        talonFXConfiguration.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        TalonFX talonFX = new TalonFX(deviceNumber, RobotConstants.rio);
        talonFX.getConfigurator().apply(talonFXConfiguration);


        ArmConstants armConstants = new ArmConstants(maxAngle, minAngle, maxAngularVelocity, maxAngularAcceleration, ks, kg, kv, ka, armLength, reduction, Radians.of(0.0), Radians.of(0.0), RadiansPerSecond.of(0.0));


        Arm arm = new KrakenX60Arm(armConstants, talonFX, cancoder);
        return new CommandArm(arm, simLoopPeriod);


    }


}
