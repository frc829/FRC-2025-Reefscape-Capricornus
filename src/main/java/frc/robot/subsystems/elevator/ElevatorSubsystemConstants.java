package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.*;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import digilib.elevator.DualVortexElevator;
import digilib.elevator.Elevator;
import digilib.elevator.ElevatorConstants;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.*;
import static com.revrobotics.spark.config.SparkBaseConfig.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorSubsystemConstants.Control.*;
import static frc.robot.subsystems.elevator.ElevatorSubsystemConstants.Mechanism.*;
import static frc.robot.subsystems.elevator.ElevatorSubsystemConstants.Simulation.*;

public class ElevatorSubsystemConstants {

    static final class Control {
        static final Voltage ks = Volts.of(0.097815);
        static final Voltage kg = Volts.of(0.18218);
        static final Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv = Volts.per(MetersPerSecond).of(15.335);
        static final Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka = Volts.per(MetersPerSecondPerSecond).of(0.36696);
        static final double positionKp = 12.106;
        static final double positionKd = 0.17223;
        static final double velocityKp = 0.10044;
        static final LinearVelocity maxVelocity = MetersPerSecond.of(
                (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / kv.baseUnitMagnitude());
        static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(
                (12.0 - ks.baseUnitMagnitude() - kg.baseUnitMagnitude()) / ka.baseUnitMagnitude());
        static final Time updatePeriod = Seconds.of(0.020);
    }

    static final class Mechanism {
        static final String name = "Elevator";
        static final double reduction = 5 * 4;
        static final Distance drumRadius = Inches.of(1.757 / 2.0);
        static final ElevatorConstants constants = new ElevatorConstants(
                name,
                reduction,
                drumRadius,
                maxHeight,
                minHeight,
                startingHeight,
                ks,
                kg,
                kv,
                ka,
                maxVelocity,
                maxAcceleration,
                positionStdDev,
                velocityStdDev);
    }

    static final class Simulation {
        static final Distance startingHeight = Centimeters.of(0.0);
        static final Distance minHeight = Centimeters.of(0.0);
        static final Distance maxHeight = Centimeters.of(65);
        static final Distance positionStdDev = Meters.of(0.0);
        static final LinearVelocity velocityStdDev = MetersPerSecond.of(0.0);
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    static final class Motor {
        static final int deviceId = 15;
        static final IdleMode idleMode = kBrake;
        static final boolean inverted = false;
        static final int depth = 2;
        static final int periodMs = 16;
        static final EncoderConfig encoderConfig = new EncoderConfig()
                .positionConversionFactor(2 * Math.PI * drumRadius.in(Meters) / reduction)
                .velocityConversionFactor(2 * Math.PI * drumRadius.in(Meters) / reduction / 60.0)
                .quadratureAverageDepth(depth)
                .quadratureMeasurementPeriod(periodMs);
        static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
                .pidf(positionKp, 0.0, positionKd, 0.0)
                .pidf(velocityKp, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1)
                .feedbackSensor(kPrimaryEncoder);
        static final SparkBaseConfig config = new SparkFlexConfig()
                .idleMode(idleMode)
                .inverted(inverted)
                .apply(encoderConfig)
                .apply(closedLoopConfig)
                .smartCurrentLimit(80);
        static final SparkFlex motor = new SparkFlex(deviceId, kBrushless);
    }

    static final class Follower {
        static final int deviceId = 25;
        static final IdleMode idleMode = kBrake;
        static final boolean inverted = true;
        static final SparkBaseConfig config = new SparkFlexConfig()
                .idleMode(idleMode)
                .follow(Motor.deviceId, inverted)
                .smartCurrentLimit(40);
        static final SparkFlex motor = new SparkFlex(deviceId, kBrushless);
    }

    public static ElevatorSubsystem create() {
        Motor.motor.configure(Motor.config, kResetSafeParameters, kPersistParameters);
        Follower.motor.configure(Follower.config, kResetSafeParameters, kPersistParameters);
        Elevator elevator = new DualVortexElevator(
                constants,
                Motor.motor,
                Follower.motor,
                updatePeriod);
        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevator, kg, simLoopPeriod);
        elevatorSubsystem.setDefaultCommand(elevatorSubsystem.hold());
        return elevatorSubsystem;
    }
}
