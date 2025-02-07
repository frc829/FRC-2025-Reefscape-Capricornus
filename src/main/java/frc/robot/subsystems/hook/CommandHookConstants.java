package frc.robot.subsystems.hook;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import digilib.arm.Arm;
import digilib.arm.ArmConstants;
import digilib.arm.NEO550Arm;
import frc.robot.subsystems.arm.CommandArm;

import static edu.wpi.first.units.Units.*;

public class CommandHookConstants {
    private static final String name = "Hook";
    private static final Angle startingAngle = Degrees.of(0.0);
    private static final Angle minAngle = Degrees.of(0.0);
    private static final Angle maxAngle = Degrees.of(90.0);
    private static final Distance armLength = Inches.of(12);
    private static final int motorDeviceId = 28;
    private static final SparkBaseConfig.IdleMode idleMode = SparkBaseConfig.IdleMode.kBrake;
    private static final boolean inverted = false;
    private static final double reduction = 12.0;
    private static final Voltage ks = Volts.of(0.0);
    private static final Voltage kg = Volts.of(0.27467940179697226);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of(8.892526274503728);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of(0.028028510387446144);
    private static final double positionKp = 27.510452235583863;
    private static final double positionKd = 0.024571536850317308;
    private static final double velocityKp = 6.183764634853723E-4;
    private static final Time updatePeriod = Seconds.of(0.020);
    private static final Time simLoopPeriod = Seconds.of(0.001);

    private static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(
            (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / kv.baseUnitMagnitude());
    private static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(
            (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / ka.baseUnitMagnitude());

    public static CommandHook createCommandHook() {
        SparkMax motor = new SparkMax(motorDeviceId, SparkLowLevel.MotorType.kBrushless);

        SparkBaseConfig config = new SparkFlexConfig()
                .idleMode(idleMode)
                .inverted(inverted);
        config.encoder.positionConversionFactor(2 * Math.PI / reduction);
        config.encoder.velocityConversionFactor(2 * Math.PI / reduction / 60.0);
        config.encoder.uvwAverageDepth(2);
        config.encoder.uvwMeasurementPeriod(16);
        config.closedLoop.pid(positionKp, 0, positionKd).feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(velocityKp, 0, 0, ClosedLoopSlot.kSlot1).feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        ArmConstants armConstants = new ArmConstants(
                name,
                maxAngle,
                minAngle,
                maxAngularVelocity,
                maxAngularAcceleration,
                ks,
                kg,
                kv,
                ka,
                armLength,
                reduction,
                startingAngle, Radians.of(0.0), RadiansPerSecond.of(0.0));
        Arm arm = new NEO550Arm(
                armConstants,
                motor,
                config,
                updatePeriod);
        return new CommandHook(arm, simLoopPeriod);


    }

}
