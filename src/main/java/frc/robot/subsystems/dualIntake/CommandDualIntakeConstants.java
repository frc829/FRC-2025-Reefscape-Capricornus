package frc.robot.subsystems.dualIntake;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import digilib.objectDetectors.LaserCanObjectDetector;
import digilib.objectDetectors.ObjectDetector;
import digilib.objectDetectors.ObjectDetectorConstants;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import digilib.intakeWheel.IntakeWheel;
import digilib.intakeWheel.IntakeWheelConstants;
import digilib.intakeWheel.NEO550IntakeWheel;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.dualIntake.CommandDualIntakeConstants.ObjectDetection.laserCanId;
import static frc.robot.subsystems.dualIntake.CommandDualIntakeConstants.ObjectDetection.rangingMode;

public class CommandDualIntakeConstants {

    static final class Algae {
        static final class Control {
            static final Voltage ks = Volts.of(0.11457);
            static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of(0.12051);
            static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of(0.0040892);
            static final double kp = 0.00077424;
            static final AngularVelocity maxVelocity = RadiansPerSecond.of(
                    (12.0 - ks.baseUnitMagnitude()) / kv.baseUnitMagnitude());
            static final AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(
                    (12.0 - ks.baseUnitMagnitude()) / ka.baseUnitMagnitude());
            static final Time updatePeriod = Seconds.of(0.020);
        }

        static final class Mechanism {
            static final String name = "Intake: Algae";
            static final double reduction = 12.0;
            static final Distance wheelRadius = Inches.of(2.0);
            static final IntakeWheelConstants constants = new IntakeWheelConstants(
                    name,
                    reduction,
                    wheelRadius,
                    Control.ks,
                    Control.kv,
                    Control.ka,
                    Control.maxVelocity,
                    Control.maxAcceleration,
                    Simulation.velocityStdDev);
        }

        static final class Simulation {
            static final AngularVelocity velocityStdDev = RadiansPerSecond.of(0.0);
            static final Time simLoopPeriod = Seconds.of(0.001);
        }

        static final class Motor {
            static final int deviceId = 16;
            static final IdleMode idleMode = kBrake;
            static final boolean inverted = false;
            static final int depth = 2;
            static final int periodMs = 16;
            static final EncoderConfig encoderConfig = new EncoderConfig()
                    .positionConversionFactor(2 * Math.PI / Mechanism.reduction)
                    .velocityConversionFactor(2 * Math.PI / Mechanism.reduction / 60.0)
                    .uvwAverageDepth(depth)
                    .uvwMeasurementPeriod(periodMs);
            static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
                    .pidf(Control.kp, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1)
                    .feedbackSensor(kPrimaryEncoder);
            static final SparkBaseConfig config = new SparkFlexConfig()
                    .idleMode(idleMode)
                    .inverted(inverted)
                    .apply(encoderConfig)
                    .apply(closedLoopConfig);
            static final SparkMax motor = new SparkMax(deviceId, kBrushless);
        }
    }

    static final class Coral {
        static final class Control {
            private static final Voltage ks = Volts.of(0.087863);
            private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of(0.11972);
            private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of(0.003406);
            private static final double kp = 0.00040594;
            private static final AngularVelocity maxVelocity = RadiansPerSecond.of(
                    (12.0 - ks.baseUnitMagnitude()) / kv.baseUnitMagnitude());
            private static final AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(
                    (12.0 - ks.baseUnitMagnitude()) / ka.baseUnitMagnitude());
            static final Time updatePeriod = Seconds.of(0.020);
        }

        static final class Mechanism {
            static final String name = "Intake: Coral";
            static final double reduction = 12.0;
            static final Distance wheelRadius = Inches.of(1.5);
            static final IntakeWheelConstants constants = new IntakeWheelConstants(
                    name,
                    reduction,
                    wheelRadius,
                    Control.ks,
                    Control.kv,
                    Control.ka,
                    Control.maxVelocity,
                    Control.maxAcceleration,
                    Simulation.velocityStdDev);
        }

        static final class Simulation {
            static final AngularVelocity velocityStdDev = RadiansPerSecond.of(0.0);
            static final Time simLoopPeriod = Seconds.of(0.001);
        }

        static final class Motor {
            static final int deviceId = 26;
            static final IdleMode idleMode = kBrake;
            static final boolean inverted = false;
            static final int depth = 2;
            static final int periodMs = 16;
            static final EncoderConfig encoderConfig = new EncoderConfig()
                    .positionConversionFactor(2 * Math.PI / Mechanism.reduction)
                    .velocityConversionFactor(2 * Math.PI / Mechanism.reduction / 60.0)
                    .uvwAverageDepth(depth)
                    .uvwMeasurementPeriod(periodMs);
            static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
                    .pidf(Control.kp, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1)
                    .feedbackSensor(kPrimaryEncoder);
            static final SparkBaseConfig config = new SparkFlexConfig()
                    .idleMode(idleMode)
                    .inverted(inverted)
                    .apply(encoderConfig)
                    .apply(closedLoopConfig);
            static final SparkMax motor = new SparkMax(deviceId, kBrushless);
        }
    }

    static final class ObjectDetection {
        static final String name = "Coral Detector";
        static final int laserCanId = 36;
        static final LaserCanInterface.RangingMode rangingMode = LaserCanInterface.RangingMode.LONG;
        static final Distance maxTrueDistance = Millimeters.of(15.0);
        static final Distance minTrueDistance = Millimeters.of(-10.0);
        static final ObjectDetectorConstants constants = new ObjectDetectorConstants(name, maxTrueDistance, minTrueDistance);
    }

    public static CommandDualIntake create() {
        Algae.Motor.motor.configure(Algae.Motor.config, kResetSafeParameters, kPersistParameters);
        Coral.Motor.motor.configure(Coral.Motor.config, kResetSafeParameters, kPersistParameters);
        IntakeWheel algaeWheel = new NEO550IntakeWheel(Algae.Mechanism.constants, Algae.Motor.motor, Algae.Control.updatePeriod);
        IntakeWheel coralWheel = new NEO550IntakeWheel(Coral.Mechanism.constants, Coral.Motor.motor, Coral.Control.updatePeriod);
        LaserCan laserCan;
        try{
            laserCan = new LaserCan(laserCanId);
            laserCan.setRangingMode(rangingMode);
        } catch (ConfigurationFailedException e) {
            laserCan = null;
        }
        ObjectDetector objectDetector = new LaserCanObjectDetector(
                ObjectDetection.name,
                laserCan,
                ObjectDetection.maxTrueDistance,
                ObjectDetection.minTrueDistance);
        return new CommandDualIntake(algaeWheel, coralWheel, objectDetector, Algae.Simulation.simLoopPeriod, Coral.Simulation.simLoopPeriod);
    }
}
