package frc.robot.constants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.*;
import frc.robot.mechanisms.elevator.DualVortexElevator;
import frc.robot.mechanisms.elevator.Elevator;
import frc.robot.mechanisms.elevator.ElevatorConstants;
import frc.robot.subsystems.CommandElevator;

import static edu.wpi.first.units.Units.*;

public class CommandElevatorConstants {
    // TODO: All of these fields are private static and final
    // TODO: Create a Distance named minHeight and assign Meters.of(0.0) to it
    // TODO: Repeat for maxAngle and set to 1 meter
    // TODO: Create a Distance named drumRadius and assign Inches.of(1.757) to it

    // TODO: create an int named primaryMotorDeviceId and assign 15 to it.
    // TODO: create an int named followerMotorDeviceId and assign 25 to it.

    // TODO: create a IdleMode named idleMode and assign IdleMode.kBrake.
    // TODO: create a boolean called primaryInverted and set to false;
    // TODO: create a double called reduction and assign 1.0 to it.

    // TODO: create a Voltage named ks and assign Volts.of(0.0) to it.
    // TODO: create a Voltage named kg and assign Volts.of(0.0) to it.

    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv = Volts.per(RadiansPerSecond).of(1.0);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka = Volts.per(RadiansPerSecondPerSecond).of(1.0);

    // TODO: create a double called positionKp and assign 0.0 to it
    // TODO: create a double called positionKd and assign 0.0 to it.
    // TODO: create a double called velocityKp and assign 0.0 to it.

    // TODO: create a Time called updatePeriod and set to Seconds.of(0.020);
    // TODO: create a Time called simLoopPeriod and set to Seconds.of(0.001);

    public static CommandElevator createCommandElevator() {
        // TODO: make a new SparkFlex named primaryMotor and initialize with primaryMotorDeviceId, and kBrushless
        // TODO: repeat with followerMotor, followerMotorDeviceId, kBrushless

        // TODO: create a SparkBaseConfig

        SparkBaseConfig primaryMotorConfig = new SparkFlexConfig();
                // .idleMode(idleMode)
                // .inverted(inverted);
        primaryMotorConfig.encoder.positionConversionFactor(1.0);
        primaryMotorConfig.encoder.velocityConversionFactor(1.0);
        primaryMotorConfig.encoder.quadratureAverageDepth(2);
        primaryMotorConfig.encoder.quadratureMeasurementPeriod(16);
        primaryMotorConfig.closedLoop.pid(0, 0, 0).feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        primaryMotorConfig.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot1).feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);

        // TODO: make a followerMotorConfig,  don't set inverted, don't set pid
        //
        // primaryMotor.configure(primaryMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        // followerMotor.configure(followerMotorconfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // TODO: create an ElevatorConstants named elevatorConstants and pass in
        // maxHeight, minHeight, ks, kg, kv, ka, drumRadius, reduction, startingHeight, positionStdDevs, velocityStdDevs

        // TODO: create an Elevator named elevator and initialize with DualVortexElevator
        // passing in elevatorConstants, primaryMotor, followerMotor, primaryMotorConfig, followerMotorConfig, updatePeriod


        // TODO: return new CommandElevator passing in elevator and simLoopPeriod
        return null; // TODO: remove this when done.


    }

}
