package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import digilib.wrist.NEO550Wrist;
import digilib.wrist.Wrist;
import digilib.wrist.WristConstants;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

public class CommandWristConstants {
    private static final Angle startingAngle = Degrees.of(0);
    private static final Angle minAngle = Degrees.of(0.0);
    private static final Angle maxAngle = Degrees.of(90.0);
    private static final int motorDeviceId = 17;
    private static final SparkBaseConfig.IdleMode idleMode = SparkBaseConfig.IdleMode.kBrake;
    private static final boolean inverted = false;
    private static final double reduction = 10.0 * 7.0 * 32.0 / 18.0;
    private static final Voltage ks = Volts.of(0.0);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of( 1.2782399036744865);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of( 9.941089837997054E-4);
    private static final double positionKp = 23.790020451150006;
    private static final double positionKd = 0.010130792769467259;
    private static final double velocityKp = 2.3496718293473E-13;
    private static final Time updatePeriod = Seconds.of(0.020);
    private static final Time simLoopPeriod = Seconds.of(0.001);

    private static final int cancoderDeviceNumber = 37;
    private static final double magnetDirection = 0.460938;

    private static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(
            (12.0 - ks.baseUnitMagnitude()) / kv.baseUnitMagnitude());
    private static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(
            (12.0 - ks.baseUnitMagnitude()) / ka.baseUnitMagnitude());



    public static CommandWrist createCommandWrist() {
        SparkMax motor = new SparkMax (motorDeviceId, SparkLowLevel.MotorType.kBrushless);

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = magnetDirection;
        CANcoder cancoder = new CANcoder(cancoderDeviceNumber, Constants.rio);
        cancoder.getConfigurator().apply(cancoderConfig);

        SparkBaseConfig config = new SparkMaxConfig()
                .idleMode(idleMode)
                .inverted(inverted);
        config.encoder.positionConversionFactor(2 * Math.PI  / reduction);
        config.encoder.velocityConversionFactor(2 * Math.PI / reduction / 60.0);
        config.encoder.uvwAverageDepth(2);
        config.encoder.uvwMeasurementPeriod(16);
        config.closedLoop.pid(positionKp, 0, positionKd).feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(velocityKp, 0, 0, ClosedLoopSlot.kSlot1).feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        WristConstants wristConstants = new WristConstants(
                maxAngle,
                minAngle,
                maxAngularVelocity,
                maxAngularAcceleration,
                ks,
                kv,
                ka,
                reduction,
                startingAngle,
                Radians.of(0.0),
                RadiansPerSecond.of(0.0));
        Wrist wrist = new NEO550Wrist(
                wristConstants,
                motor,
                config,
                cancoder,
                updatePeriod);
        return new CommandWrist(wrist, simLoopPeriod);


    }

}
