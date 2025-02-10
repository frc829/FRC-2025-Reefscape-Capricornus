package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import digilib.elevator.DualVortexElevator;
import digilib.elevator.Elevator;
import digilib.elevator.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

public class CommandElevatorConstants {

    private static final String name = "Elevator";
    private static final Distance startingHeight = Feet.of(0.0);
    private static final Distance minHeight = Feet.of(0.0);
    private static final Distance maxHeight = Meters.of(4.0);
    private static final Distance drumRadius = Inches.of(1.757).times(2); // because 2 stages
    private static final int primaryMotorDeviceId = 15;
    private static final int followerMotorDeviceId = 25;
    private static final SparkBaseConfig.IdleMode idleMode = SparkBaseConfig.IdleMode.kBrake;
    private static final boolean primaryInverted = false;
    private static final double reduction = 5 * 4;
    private static final Voltage ks = Volts.of(0.08335);
    private static final Voltage kg = Volts.of(0.18624);
    private static final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv = Volts.per(MetersPerSecond).of(3.9235);
    private static final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka = Volts.per(MetersPerSecondPerSecond).of(0.08335);
    private static final double positionKp = 3.4894;
    private static final double positionKd = 0.21401;
    private static final double velocityKp = 0.019134;
    private static final Time updatePeriod = Seconds.of(0.020);
    private static final Time simLoopPeriod = Seconds.of(0.001);
    private static final LinearVelocity maxVelocity = MetersPerSecond.of(
            (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / kv.baseUnitMagnitude());
    private static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(
            (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / ka.baseUnitMagnitude());

    public static CommandElevator createCommandElevator() {
        SparkFlex primaryMotor = new SparkFlex(primaryMotorDeviceId, SparkLowLevel.MotorType.kBrushless);
        SparkFlex followerMotor = new SparkFlex(followerMotorDeviceId, SparkLowLevel.MotorType.kBrushless);

        SparkBaseConfig primaryMotorConfig = new SparkFlexConfig()
                .idleMode(idleMode)
                .inverted(primaryInverted);
        primaryMotorConfig.encoder.positionConversionFactor(2 * Math.PI * drumRadius.in(Meters) / reduction);
        primaryMotorConfig.encoder.velocityConversionFactor(2 * Math.PI * drumRadius.in(Meters) / reduction / 60.0);
        primaryMotorConfig.encoder.quadratureAverageDepth(2);
        primaryMotorConfig.encoder.quadratureMeasurementPeriod(16);
        primaryMotorConfig.closedLoop
                .pid(positionKp, 0, positionKd)
                .velocityFF(0.0)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1, 1);
        primaryMotorConfig.closedLoop
                .pid(velocityKp, 0, 0, ClosedLoopSlot.kSlot1)
                .velocityFF(0.0, ClosedLoopSlot.kSlot1)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        SparkBaseConfig followerMotorConfig = new SparkFlexConfig()
                .idleMode(idleMode);
        followerMotorConfig.follow(primaryMotor, true);

        primaryMotor.configure(primaryMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        followerMotor.configure(followerMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        ElevatorConstants elevatorConstants = new ElevatorConstants(
                name,
                maxHeight,
                minHeight,
                maxVelocity,
                maxAcceleration,
                ks,
                kg,
                kv,
                ka,
                drumRadius,
                reduction,
                startingHeight, Meters.of(0.0), MetersPerSecond.of(0.0));
        Elevator elevator = new DualVortexElevator(
                elevatorConstants,
                primaryMotor,
                followerMotor,
                primaryMotorConfig,
                followerMotorConfig,
                updatePeriod);
        return new CommandElevator(elevator, simLoopPeriod);


    }

}
