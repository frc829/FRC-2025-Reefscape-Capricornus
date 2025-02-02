package frc.robot.constants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.mechanisms.elevator.DualVortexElevator;
import frc.robot.mechanisms.elevator.Elevator;
import frc.robot.mechanisms.elevator.ElevatorConstants;
import frc.robot.subsystems.CommandElevator;

import static edu.wpi.first.units.Units.*;

public class CommandElevatorConstants {
    private static final Distance minHeight = Meters.of(0.0);
    private static final Distance maxHeight = Meters.of(1.0);
    private static final Distance drumRadius = Inches.of(1.757);
    private static final int primaryMototrDeviceId = 15;
    private static final int followerMotorDeviceId = 25;
    private static final SparkBaseConfig.IdleMode idleMode = SparkBaseConfig.IdleMode.kBrake;
    private static final boolean primaryInverted = false;
    private static final double reduction = 1.0;
    private static final Voltage ks = Volts.of(0.0);
    private static final Voltage kg = Volts.of(0.0);
    private static final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv = Volts.per(MetersPerSecond).of(1.0);
    private static final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka = Volts.per(MetersPerSecondPerSecond).of(1.0);
    private static final double positionKp = 0.0;
    private static final double positionKd = 0.0;
    private static final double velocityKp = 0.0;
    private static final Time updatePeriod = Seconds.of(0.020);
    private static final Time simLoopPeriod = Seconds.of(0.001);

    public static CommandElevator createCommandElevator() {
        SparkFlex primaryMotor = new SparkFlex (primaryMototrDeviceId, SparkLowLevel.MotorType.kBrushless);
        SparkFlex followerMotor = new SparkFlex(followerMotorDeviceId, SparkLowLevel.MotorType.kBrushless);



        SparkBaseConfig primaryMotorConfig = new SparkFlexConfig()
                .idleMode(idleMode)
                .inverted(primaryInverted);
        primaryMotorConfig.encoder.positionConversionFactor(1.0);
        primaryMotorConfig.encoder.velocityConversionFactor(1.0);
        primaryMotorConfig.encoder.quadratureAverageDepth(2);
        primaryMotorConfig.encoder.quadratureMeasurementPeriod(16);
        primaryMotorConfig.closedLoop.pid(0, 0, 0).feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        primaryMotorConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot1).feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        SparkBaseConfig followerMotorconfig = new SparkFlexConfig()
                .idleMode(idleMode);
        followerMotorconfig.encoder.positionConversionFactor(1.0);
        followerMotorconfig.encoder.velocityConversionFactor(1.0);
        followerMotorconfig.encoder.quadratureAverageDepth(2);
        followerMotorconfig.encoder.quadratureMeasurementPeriod(16);

        primaryMotor.configure(primaryMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        followerMotor.configure(followerMotorconfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        ElevatorConstants elevatorConstants = new ElevatorConstants(maxHeight, minHeight, ks, kg, kv, ka, drumRadius, reduction, Meters.of(0.0), Meters.of(0.0), MetersPerSecond.of(0.0));
        Elevator elevator = new DualVortexElevator(elevatorConstants, primaryMotor, followerMotor, primaryMotorConfig, followerMotorconfig, updatePeriod);
        return new CommandElevator(elevator, simLoopPeriod);


    }

}
