package digilib.intakeWheel;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;

public class IntakeSubsystemConstants {

    static final String name = "Intake: Coral";
    static final double reduction = 12.0;
    static final double ksVolts = 0.12198;
    static final double kvVoltsPerRPS = 0.12267 * 2 * Math.PI;
    static final double kaVoltsPerRPSSquared = 0.0045787 * 2 * Math.PI;
    static final double kpVoltsPerRPS = 0.00011686 * 2 * Math.PI;
    static final double maxControlVoltage = 12.0 - ksVolts;
    static final double maxVelocityRPS = maxControlVoltage / kvVoltsPerRPS;
    static final double maxAccelerationRPSS = maxControlVoltage / kaVoltsPerRPSSquared;
    static final double controlPeriodSeconds = 0.020;
    static final SlewRateLimiter profile = new SlewRateLimiter(maxAccelerationRPSS);
    static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            ksVolts, kvVoltsPerRPS, kaVoltsPerRPSSquared, controlPeriodSeconds);
    static final DCMotor dcMotor = DCMotor.getNeo550(1);
    static final LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(kvVoltsPerRPS / 2 / Math.PI, kaVoltsPerRPSSquared / 2 / Math.PI);
    static final DCMotorSim dcMotorSim = new DCMotorSim(plant, dcMotor);

    static final int deviceId = 26;
    static final IdleMode idleMode = kBrake;
    static final boolean inverted = false;
    static final int smartCurrentLimit = 20;
    static final int depth = 2;
    static final int periodMs = 16;
    static final double positionFactor = 1.0 / reduction;
    static final double velocityFactor = 1.0 / reduction / 60.0;
    static final SignalsConfig signalsConfig = new SignalsConfig()
            .absoluteEncoderPositionAlwaysOn(false)
            .absoluteEncoderVelocityAlwaysOn(false)
            .analogPositionAlwaysOn(false)
            .analogVelocityAlwaysOn(false)
            .externalOrAltEncoderPositionAlwaysOn(false)
            .externalOrAltEncoderVelocityAlwaysOn(false)
            .primaryEncoderPositionAlwaysOn(false)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20);
    static final EncoderConfig encoderConfig = new EncoderConfig()
            .positionConversionFactor(positionFactor)
            .velocityConversionFactor(velocityFactor)
            .uvwAverageDepth(depth)
            .uvwMeasurementPeriod(periodMs);
    static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
            .p(kpVoltsPerRPS, ClosedLoopSlot.kSlot1)
            .feedbackSensor(kPrimaryEncoder);
    static final SparkBaseConfig config = new SparkFlexConfig()
            .idleMode(idleMode)
            .inverted(inverted)
            .smartCurrentLimit(smartCurrentLimit)
            .apply(signalsConfig)
            .apply(encoderConfig)
            .apply(closedLoopConfig);
    static final SparkMax motor = new SparkMax(deviceId, kBrushless);
    static final REVLibError revLibError = motor.configure(config, kResetSafeParameters, kPersistParameters);
    static final SparkMaxSim motorSim = new SparkMaxSim(motor, dcMotor);
    static final Time simLoopPeriod = Seconds.of(0.001);
    static final IntakeWheel intakeWheel = new SparkMaxIntakeWheel(
            name, maxVelocityRPS, maxAccelerationRPSS, motor, profile, feedforward, dcMotorSim, motorSim);


    public static IntakeWheelSubsystem create() {
        IntakeWheelSubsystem intakeWheelSubsystem = new IntakeWheelSubsystem(intakeWheel, simLoopPeriod);
        intakeWheelSubsystem.setDefaultCommand(intakeWheelSubsystem.toVoltage(0.0));
        return intakeWheelSubsystem;
    }
}
