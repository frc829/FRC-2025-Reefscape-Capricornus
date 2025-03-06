package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import digilib.elevator.TwoVortexElevator;
import digilib.elevator.Elevator;
import digilib.elevator.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;

import static com.revrobotics.spark.ClosedLoopSlot.*;
import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.*;
import static com.revrobotics.spark.config.SparkBaseConfig.*;
import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.elevator.ElevatorSubsystemConstants.Control.*;
import static frc.robot.subsystems.elevator.ElevatorSubsystemConstants.Mechanism.*;
import static frc.robot.subsystems.elevator.ElevatorSubsystemConstants.Simulation.*;

public class ElevatorSubsystemConstants {

    static final class Control {
        static final double ksVolts = 0.10711;
        static final double kgVolts = 0.18368;
        static final double kvVoltsPerMPS = 15.234;
        static final double kaVoltsPerMPSSquared = 0.36189;
        static final double positionKpVoltsPerMeter = 2.1935;
        static final double positionKdVoltsPerMPS = 0.22279;
        static final double velocityKpVoltsPerMPS = 0.19185;
        static final double maxControlVoltage = 12.0 - ksVolts - kgVolts;
        static final double maxVelocityMPS = maxControlVoltage / kvVoltsPerMPS;
        static final double maxAccelerationMPSS = maxControlVoltage / kaVoltsPerMPSSquared;
        static final double controlPeriodSeconds = 0.020;
    }

    static final class Simulation {
        static final double startingHeightMeters = 0.0;
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    static final class Mechanism {
        static final String name = "Elevator";
        static final double reduction = 5 * 4;
        static final double sprocketPitchDiameterInches = 1.757;
        static final double sprocketPitchRadiusInches = sprocketPitchDiameterInches / 2;
        static final double drumRadiusMeters = Units.inchesToMeters(sprocketPitchRadiusInches);
        static final double minHeightCentimeters = 0.0;
        static final double minHeightMeters = minHeightCentimeters / 100.0;
        static final double maxHeightCentimeters = 65;
        static final double maxHeightMeters = maxHeightCentimeters / 100.0;
        static final ElevatorConstants constants = new ElevatorConstants(
                name,
                reduction,
                drumRadiusMeters,
                startingHeightMeters,
                minHeightMeters,
                maxHeightMeters,
                maxControlVoltage,
                ksVolts,
                kgVolts,
                kvVoltsPerMPS,
                kaVoltsPerMPSSquared,
                maxVelocityMPS,
                maxAccelerationMPSS);
    }

    static final class Motor {
        static final int deviceId = 15;
        static final IdleMode idleMode = kBrake;
        static final boolean inverted = false;
        static final int smartCurrentLimit = 80;
        static final int depth = 2;
        static final int periodMs = 16;
        static final double positionFactor = 2 * Math.PI * drumRadiusMeters / reduction;
        static final double velocityFactor = 2 * Math.PI * drumRadiusMeters / reduction / 60.0;
        static final SignalsConfig signalsConfig = new SignalsConfig()
                .absoluteEncoderPositionAlwaysOn(false)
                .absoluteEncoderVelocityAlwaysOn(false)
                .analogPositionAlwaysOn(false)
                .analogVelocityAlwaysOn(false)
                .externalOrAltEncoderPositionAlwaysOn(false)
                .externalOrAltEncoderVelocityAlwaysOn(false)
                .primaryEncoderPositionAlwaysOn(false)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20);
        static final EncoderConfig encoderConfig = new EncoderConfig()
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(velocityFactor)
                .quadratureAverageDepth(depth)
                .quadratureMeasurementPeriod(periodMs);
        static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
                .p(positionKpVoltsPerMeter)
                .d(positionKdVoltsPerMPS)
                .p(velocityKpVoltsPerMPS, kSlot1)
                .feedbackSensor(kPrimaryEncoder);
        static final SparkBaseConfig config = new SparkFlexConfig()
                .idleMode(idleMode)
                .inverted(inverted)
                .smartCurrentLimit(smartCurrentLimit)
                .apply(signalsConfig)
                .apply(encoderConfig)
                .apply(closedLoopConfig);
        static final SparkFlex motor = new SparkFlex(deviceId, kBrushless);
    }

    static final class Follower {
        static final int deviceId = 25;
        static final IdleMode idleMode = kBrake;
        static final boolean inverted = true;
        static final int smartCurrentLimit = 80;
        static final SparkBaseConfig config = new SparkFlexConfig()
                .idleMode(idleMode)
                .follow(Motor.deviceId, inverted)
                .smartCurrentLimit(smartCurrentLimit);
        static final SparkFlex motor = new SparkFlex(deviceId, kBrushless);
    }

    public static ElevatorSubsystem create() {
        Motor.motor.configure(Motor.config, kResetSafeParameters, kPersistParameters);
        Follower.motor.configure(Follower.config, kResetSafeParameters, kPersistParameters);
        Elevator elevator = new TwoVortexElevator(
                constants,
                Motor.motor,
                Follower.motor,
                controlPeriodSeconds);
        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevator, simLoopPeriod);
        MutDistance holdPosition = Meters.mutable(0.0);
        Command hold = runOnce(() -> holdPosition.mut_setBaseUnitMagnitude(elevator.getState().getMotorEncoderPositionMeters()))
                        .andThen(elevatorSubsystem.toHeight(holdPosition.baseUnitMagnitude()));
        elevatorSubsystem.setDefaultCommand(hold);
        return elevatorSubsystem;
    }
}
