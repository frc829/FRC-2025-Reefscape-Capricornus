package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.subsystems.CommandArm;

public class CommandArmConstants {
    // TODO: All of these fields are private static and final
    // TODO: Create an Angle named minAngle and assign Degrees.of(-20) to it
    // TODO: Repeat for maxAngle and set to 20 degrees
    // TODO: Create a Distance named armLength and assign Meters.of(1.0) to it

    // TODO: create an int named deviceNumber and assign 14 to it.
    // TODO: create a NeutralModeValue named neutralModeValue and assign NeutralModeValue.Brake to it.
    // TODO: create an InvertedValue named invertedValue and assign InvertedValue.CounterClockwise_Positive to it.
    // TODO: create a double called reduction and assign 1.0 to it.

    // TODO: create a Voltage named ks and assign Volts.of(0.0) to it.
    // TODO: create a Voltage named kg and assign Volts.of(0.0) to it.

    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of(1.0);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of(1.0);

    // TODO: create a double called positionKp and assign 0.0 to it
    // TODO: create a double called positionKd and assign 0.0 to it.
    // TODO: create a double called velocityKp and assign 0.0 to it.

    // TODO: create an int named canCoderDeviceNumber and assign 34 to it.
    // TODO: create a double magnetDirection and assign 0.0 to it.
    // TODO: create a Time called simLoopPeriod and set to Seconds.of(0.001);



    public static CommandArm createCommandArm() {
        // TODO: create a CANcoderConfiguration called cancoderConfig and initialize using the default constructor
        // TODO: set cancoderConfig.MagnetSensor.SensorDirection and intiailize to SensorDirectionValue.CounterClockwise_Positive
        // TODO: set cancoderConfig.MagnetSensor.MagnetOffset to magnetDirection;
        // TODO: create a new CANcoder called cancoder and initialize with cancoderDeviceNumber and UniversalRobotConstants.rio
        // TODO: call cancoder's getConfigurator().apply method and pass cancoderConfig

        // TODO: set talonFXConfiguration.Voltage.PeakForwardVoltage to 12.0
        // TODO: set talonFXConfiguration.Voltage.PeakReverseVoltage to -12.0
        // TODO: set talonFXConfiguration.ClosedLoopGeneral.ContinuousWrap to true
        // TODO: set talonFXConfiguration.Feedback.FeedbackRemoteSensorId to cancoderDeviceNumber
        // TODO: set talonFXConfiguration.Feedback.FeedbackSensorSource to FeedbackSensorSourceValue.SyncCANcoder
        // TODO: set talonFXConfiguration.Feedback.RotorToSensorRatio reduction
        // TODO: set talonFXConfiguration.Feedback.SensorToMechanismRatio to 1.0
        // TODO: set talonFXConfiguration.MotionMagic.MotionMagicExpo_kV to kv.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.MotionMagic.MotionMagicExpo_kA to ka.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.MotorOutput.Inverted to invertedValue
        // TODO: set talonFXConfiguration.MotorOutput.NeutralMode to neutralModeValue
        // TODO: set talonFXConfiguration.Slot0.GravityType to GravityTypeValue.Arm_Cosine
        // TODO: set talonFXConfiguration.Slot0.kP to positionKp
        // TODO: set talonFXConfiguration.Slot0.kD to positionKd
        // TODO: set talonFXConfiguration.Slot0.kS to ks.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.Slot0.kV to kv.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.Slot0.kA to ka.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.Slot0.kG to kg.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.Slot0.StaticFeedforwardSign to StaticFeedforwardSignValue.UseClosedLoopSign;
        // TODO: set talonFXConfiguration.Slot1.kP to velocityKp;
        // TODO: set talonFXConfiguration.Slot1.GravityType to GravityTypeValue.Arm_Cosine
        // TODO: set talonFXConfiguration.Slot1.kS to ks.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.Slot1.kV to kv.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.Slot1.kA to ka.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.Slot1.kG to kg.baseUnitMagnitude()
        // TODO: set talonFXConfiguration.Slot1.StaticFeedforwardSign to StaticFeedforwardSignValue.UseVelocitySign;
        // TODO: create a TalonFX called talonFX and use the constructor accepting talonFXDeviceNumber and UniversalConstants.rio
        // TODO: call talonFX.getConfigurator()'s apply method and pass in talonFXConfiguration

        // TODO: create an ArmConstants called armConstants and use
        // maxAngle, minAngle, ks, kg, kv, ka, armLength, reduction, Radians.of(0.0), Radians.of(0.0), RadiansPerSecond.of(0.0) as its arguments

        // TODO: create an Arm called arm and assign a new KrakenX60Arm to its passing in armConstants, talonFX, and cancoder
        // TODO: return new CommandArm(arm, simLoopPeriod)

        return null; // TODO: remove this when done.
    }


}
