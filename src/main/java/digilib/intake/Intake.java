package digilib.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import digilib.motorControllers.MotorController;
import digilib.motorControllers.ctre.TalonFXController;
import digilib.motorControllers.rev.GearboxSparkController;
import digilib.motorControllers.rev.SparkController;
import digilib.units.currentperangacc.CurrentPerAngularAcceleration;
import digilib.units.currentperangvel.CurrentPerAngularVelocity;
import digilib.units.voltsperangacc.VoltsPerAngularAcceleration;
import digilib.units.voltsperangvel.VoltsPerAngularVelocity;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static digilib.motorControllers.ctre.TalonFXController.*;
import static digilib.motorControllers.ctre.TalonFXController.position_voltage_slot;
import static digilib.motorControllers.ctre.TalonFXController.velocity_voltage_slot;
import static digilib.motorControllers.rev.SparkController.*;
import static digilib.units.currentperangacc.CurrentPerAngularAccelerationUnit.*;
import static digilib.units.currentperangvel.CurrentPerAngularVelocityUnit.*;
import static digilib.units.voltsperangacc.VoltsPerAngularAccelerationUnit.VoltsPerRadiansPerSecondPerSecond;
import static digilib.units.voltsperangacc.VoltsPerAngularAccelerationUnit.VoltsPerRotationsPerSecondPerSecond;
import static digilib.units.voltsperangvel.VoltsPerAngularVelocityUnit.*;
import static edu.wpi.first.units.Units.*;

public class Intake {

    private final MotorController motorController;

    private Intake(MotorController motorController) {
        this.motorController = motorController;
    }

    public double getVoltageVolts() {
        return motorController.getVoltageVolts();
    }

    public double getCurrentAmps() {
        return motorController.getCurrentAmps();
    }

    public double getVelocityRadPerSecond() {
        return motorController.getVelocity();
    }

    public double getAccelerationRadPerSecondSq() {
        return motorController.getAcceleration();
    }

    public void applyVoltage(double voltageVolts) {
        motorController.applyVoltage(voltageVolts);
    }

    public void applyCurrent(double currentAmps) {
        motorController.applyCurrent(currentAmps);
    }

    public void applyVelocityUsingVoltage(double velocityRadPerSecond) {
        motorController.applyVelocityUsingVoltage(velocityRadPerSecond);
    }

    public void applyVelocityUsingCurrent(double velocityRadPerSecond) {
        motorController.applyVelocityUsingCurrent(velocityRadPerSecond);
    }

    public static class TalonFXBuilder {
        private final String name;
        private final String motor;
        private final int deviceNumber;
        private final CANBus canbus;
        private final double reduction;
        private final NeutralModeValue neutralModeValue;
        private final InvertedValue invertedValue;
        private final double maxAccelerationScalar;
        private final Voltage maxControlVoltage;
        private final Current maxControlCurrent;
        private final Voltage voltageKs;
        private final VoltsPerAngularVelocity voltageKv;
        private final VoltsPerAngularAcceleration voltageKa;
        private final VoltsPerAngularVelocity velocityVoltageKp;
        private final Current currentKs;
        private final CurrentPerAngularVelocity currentKv;
        private final CurrentPerAngularAcceleration currentKa;
        private final CurrentPerAngularVelocity velocityCurrentKp;

        private Intake intake = null;

        public TalonFXBuilder(
                String name,
                String motor,
                int deviceNumber,
                CANBus canbus,
                double reduction,
                NeutralModeValue neutralModeValue,
                InvertedValue invertedValue,
                double maxAccelerationScalar,
                Voltage maxControlVoltage,
                Current maxControlCurrent,
                Voltage voltageKs,
                VoltsPerAngularVelocity voltageKv,
                VoltsPerAngularAcceleration voltageKa,
                VoltsPerAngularVelocity velocityVoltageKp,
                Current currentKs,
                CurrentPerAngularVelocity currentKv,
                CurrentPerAngularAcceleration currentKa,
                CurrentPerAngularVelocity velocityCurrentKp
        ) {
            this.name = name;
            this.motor = motor;
            this.deviceNumber = deviceNumber;
            this.canbus = canbus;
            this.reduction = reduction;
            this.neutralModeValue = neutralModeValue;
            this.invertedValue = invertedValue;
            this.maxAccelerationScalar = maxAccelerationScalar;
            this.maxControlVoltage = maxControlVoltage;
            this.maxControlCurrent = maxControlCurrent;
            this.voltageKs = voltageKs;
            this.voltageKv = voltageKv;
            this.voltageKa = voltageKa;
            this.velocityVoltageKp = velocityVoltageKp;
            this.currentKs = currentKs;
            this.currentKv = currentKv;
            this.currentKa = currentKa;
            this.velocityCurrentKp = velocityCurrentKp;
        }

        public Intake getIntake() {
            if (intake == null) {
                makeIntake();
            }
            return intake;
        }

        private void makeIntake() {
            intake = new Intake(makeMotorController());
        }

        private MotorController makeMotorController() {
            TalonFX talonFX = makeTalonFX();
            MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0)
                    .withEnableFOC(true)
                    .withSlot(0);
            MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityTorqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0.0)
                    .withSlot(1);
            return new TalonFXController(
                    talonFX,
                    null,
                    motionMagicVelocityVoltage,
                    null,
                    motionMagicVelocityTorqueCurrentFOC);
        }

        private TalonFX makeTalonFX() {
            MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                    .withInverted(invertedValue)
                    .withNeutralMode(neutralModeValue);
            VoltageConfigs voltageConfigs = new VoltageConfigs()
                    .withPeakForwardVoltage(maxControlVoltage)
                    .withPeakReverseVoltage(maxControlVoltage.unaryMinus());
            TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(maxControlCurrent)
                    .withPeakReverseTorqueCurrent(maxControlCurrent.unaryMinus());
            Slot0Configs slot0Configs = new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                    .withKS(voltageKs.in(Volts))
                    .withKV(voltageKv.in(VoltsPerRotationsPerSecond))
                    .withKA(voltageKa.in(VoltsPerRotationsPerSecondPerSecond))
                    .withKP(velocityVoltageKp.in(VoltsPerRotationsPerSecond))
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKG(0.0);
            Slot1Configs slot1Configs = new Slot1Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                    .withKS(currentKs.in(Amps))
                    .withKV(currentKv.in(AmpsPerRotationsPerSecond))
                    .withKA(currentKa.in(AmpsPerRotationsPerSecondPerSecond))
                    .withKP(velocityCurrentKp.in(AmpsPerRotationsPerSecond))
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKG(0.0);
            FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(reduction);
            MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                    .withMotionMagicAcceleration(maxAccelerationScalar * ((12 - voltageKs.in(Volts)) / voltageKa.in(VoltsPerRotationsPerSecondPerSecond)));


            TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration()
                    .withVoltage(voltageConfigs)
                    .withTorqueCurrent(torqueCurrentConfigs)
                    .withMotorOutput(motorOutputConfigs)
                    .withSlot0(slot0Configs)
                    .withSlot1(slot1Configs)
                    .withFeedback(feedbackConfigs)
                    .withMotionMagic(motionMagicConfigs);

            TalonFX talonFX = new TalonFX(deviceNumber, canbus);
            talonFX.getConfigurator().apply(talonFXConfiguration);
            return talonFX;
        }

    }

    public static class SparkBuilder {
        private final String name;
        private final String motor;
        private final int deviceId;
        private final MotorType motorType;
        private final SparkModel sparkModel;
        private final double reduction;
        private final IdleMode idleMode;
        private final boolean inverted;
        private final int depth;
        private final Time encoderPeriod;
        private final FeedbackSensor feedbackSensor;
        private final double maxAccelerationScalar;
        private final Time controlPeriod;
        private final Voltage maxControlVoltage;
        private final Current maxControlCurrent;
        private final Voltage voltageKs;
        private final VoltsPerAngularVelocity voltageKv;
        private final VoltsPerAngularAcceleration voltageKa;
        private final VoltsPerAngularVelocity velocityVoltageKp;
        private final Current currentKs;
        private final CurrentPerAngularVelocity currentKv;
        private final CurrentPerAngularAcceleration currentKa;
        private final CurrentPerAngularVelocity velocityCurrentKp;

        private Intake intake = null;

        public SparkBuilder(
                String name,
                String motor,
                int deviceId,
                MotorType motorType,
                SparkModel sparkModel,
                double reduction,
                IdleMode idleMode,
                boolean inverted,
                int depth,
                Time encoderPeriod,
                FeedbackSensor feedbackSensor,
                double maxAccelerationScalar,
                Time controlPeriod,
                Voltage maxControlVoltage,
                Current maxControlCurrent,
                Voltage voltageKs,
                VoltsPerAngularVelocity voltageKv,
                VoltsPerAngularAcceleration voltageKa,
                VoltsPerAngularVelocity velocityVoltageKp,
                Current currentKs,
                CurrentPerAngularVelocity currentKv,
                CurrentPerAngularAcceleration currentKa,
                CurrentPerAngularVelocity velocityCurrentKp) {
            this.name = name;
            this.motor = motor;
            this.deviceId = deviceId;
            this.motorType = motorType;
            this.sparkModel = sparkModel;
            this.reduction = reduction;
            this.idleMode = idleMode;
            this.inverted = inverted;
            this.depth = depth;
            this.encoderPeriod = encoderPeriod;
            this.feedbackSensor = feedbackSensor;
            this.maxAccelerationScalar = maxAccelerationScalar;
            this.controlPeriod = controlPeriod;
            this.maxControlVoltage = maxControlVoltage;
            this.maxControlCurrent = maxControlCurrent;
            this.voltageKs = voltageKs;
            this.voltageKv = voltageKv;
            this.voltageKa = voltageKa;
            this.velocityVoltageKp = velocityVoltageKp;
            this.currentKs = currentKs;
            this.currentKv = currentKv;
            this.currentKa = currentKa;
            this.velocityCurrentKp = velocityCurrentKp;
        }

        public Intake getIntake() {
            if (intake == null) {
                makeIntake();
            }
            return intake;
        }

        private void makeIntake() {
            intake = new Intake(makeMotorController());
        }

        private MotorController makeMotorController() {
            SparkBase sparkBase = makeSparkBase();
            SimpleMotorFeedforward voltageFeedforward = new SimpleMotorFeedforward(
                    voltageKs.in(Volts),
                    voltageKv.in(VoltsPerRadiansPerSecond),
                    voltageKa.in(VoltsPerRadiansPerSecondPerSecond),
                    controlPeriod.in(Seconds));
            SimpleMotorFeedforward currentFeedforward = new SimpleMotorFeedforward(
                    currentKs.in(Amps),
                    currentKv.in(AmpsPerRadiansPerSecond),
                    currentKa.in(AmpsPerRadiansPerSecondPerSecond),
                    controlPeriod.in(Seconds));
            double maxAcceleration = voltageFeedforward.maxAchievableAcceleration(maxControlVoltage.in(Volts), 0.0);
            TrapezoidProfile.Constraints velocityMotionProfileConstraints = new TrapezoidProfile.Constraints(maxAccelerationScalar * maxAcceleration, 1e9);
            TrapezoidProfile velocityMotionProfile = new TrapezoidProfile(velocityMotionProfileConstraints);
            PIDController velocityCurrentClosedLoopController = new PIDController(
                    velocityCurrentKp.in(AmpsPerRadiansPerSecond),
                    0.0,
                    0.0,
                    controlPeriod.in(Seconds));
            return new GearboxSparkController(
                    sparkBase,
                    sparkBase.getEncoder(),
                    sparkBase.getClosedLoopController(),
                    null,
                    velocityMotionProfile,
                    null,
                    velocityCurrentClosedLoopController,
                    voltageFeedforward,
                    currentFeedforward);
        }

        private SparkBase makeSparkBase() {
            EncoderConfig encoderConfig = switch (sparkModel) {
                case SparkFlex -> new EncoderConfig()
                        .quadratureAverageDepth(depth)
                        .quadratureMeasurementPeriod((int) encoderPeriod.in(Milliseconds));
                default -> new EncoderConfig()
                        .uvwAverageDepth(depth)
                        .uvwMeasurementPeriod((int) encoderPeriod.in(Milliseconds));
            };

            encoderConfig
                    .velocityConversionFactor(2 * Math.PI / reduction / 60.0);

            ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
                    .pid(
                            velocityVoltageKp.in(VoltsPerRadiansPerSecond),
                            0.0,
                            0.0,
                            SparkController.velocity_voltage_slot
                    )
                    .feedbackSensor(feedbackSensor)
                    .maxOutput(maxControlVoltage.in(Volts) / 12.0);

            SparkBaseConfig sparkBaseConfig = switch (sparkModel) {
                case SparkFlex -> new SparkFlexConfig();
                default -> new SparkMaxConfig();
            };

            sparkBaseConfig
                    .apply(encoderConfig)
                    .apply(closedLoopConfig)
                    .inverted(inverted)
                    .idleMode(idleMode)
                    .smartCurrentLimit((int) maxControlCurrent.in(Amps));

            SparkBase spark = switch (sparkModel) {
                case SparkFlex -> new SparkFlex(deviceId, motorType);
                default -> new SparkMax(deviceId, motorType);
            };

            spark.configure(sparkBaseConfig, kResetSafeParameters, kPersistParameters);
            return spark;
        }
    }
}
