package frc.robot.subsystems.dualIntake;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import digilib.objectDetectors.LaserCanObjectDetector;
import digilib.objectDetectors.ObjectDetector;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import digilib.intakeWheel.IntakeWheel;
import digilib.intakeWheel.IntakeWheelConstants;
import digilib.intakeWheel.NEO550IntakeWheel;

import static edu.wpi.first.units.Units.*;

public class CommandDualIntakeConstants {
    private static final Distance algaeWheelRadius = Inches.of(2.0);
    private static final Distance coralWheelRadius = Inches.of(1.5);
    private static final int algaeDeviceNumber = 16;
    private static final int coralDeviceNumber = 26;
    private static final SparkBaseConfig.IdleMode idleMode = SparkBaseConfig.IdleMode.kBrake;
    private static final boolean algaeInverted = false;
    private static final boolean coralInverted = false;
    private static final double reduction = 12.0;
    private static final Voltage algaeKs = Volts.of(0.11457);
    private static final Voltage coralKs = Volts.of(0.087863);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> algaeKv = Volts.per(RadiansPerSecond).of(0.12051);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> coralKv = Volts.per(RadiansPerSecond).of(0.11972);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> algaeKa = Volts.per(RadiansPerSecondPerSecond).of(0.0040892);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> coralKa = Volts.per(RadiansPerSecondPerSecond).of(0.003406);
    private static final double algaeKp = 0.00077424;
    private static final double coralKp = 0.00040594;
    private static final AngularVelocity algaeMaxVelocity = RadiansPerSecond.of(
            (12.0 - algaeKs.baseUnitMagnitude()) / algaeKv.baseUnitMagnitude());
    private static final AngularVelocity coralMaxVelocity = RadiansPerSecond.of(
            (12.0 - coralKs.baseUnitMagnitude()) / coralKv.baseUnitMagnitude());
    private static final AngularAcceleration algaeMaxAcceleration = RadiansPerSecondPerSecond.of(
            (12.0 - algaeKs.baseUnitMagnitude()) / algaeKa.baseUnitMagnitude());
    private static final AngularAcceleration coralMaxAcceleration = RadiansPerSecondPerSecond.of(
            (12.0 - coralKs.baseUnitMagnitude()) / coralKa.baseUnitMagnitude());
    private static final Time updatePeriod = Seconds.of(0.020);
    private static final Time simLoopPeriod = Seconds.of(0.001);

    private static final int laserCanId = 36;
    private static final LaserCanInterface.RangingMode rangingMode = LaserCanInterface.RangingMode.LONG;
    private static final Distance maxTrueDistance = Millimeters.of(15.0);
    private static final Distance minTrueDistance = Millimeters.of(-10.0);

    public static CommandDualIntake createCommandIntake() {
        SparkMax algaeMotor = new SparkMax(algaeDeviceNumber, SparkLowLevel.MotorType.kBrushless);
        SparkMax coralMotor = new SparkMax(coralDeviceNumber, SparkLowLevel.MotorType.kBrushless);

        SparkBaseConfig algaeConfig = new SparkMaxConfig()
                .idleMode(idleMode)
                .inverted(algaeInverted);
        SparkBaseConfig coralConfig = new SparkMaxConfig()
                .idleMode(idleMode)
                .inverted(coralInverted);

        algaeConfig.encoder.positionConversionFactor(2 * Math.PI / reduction);
        coralConfig.encoder.positionConversionFactor(2 * Math.PI / reduction);
        algaeConfig.encoder.velocityConversionFactor(2 * Math.PI / reduction / 60.0);
        coralConfig.encoder.velocityConversionFactor(2 * Math.PI / reduction / 60.0);
        algaeConfig.encoder.uvwAverageDepth(2);
        coralConfig.encoder.uvwAverageDepth(2);
        algaeConfig.encoder.uvwMeasurementPeriod(16);
        coralConfig.encoder.uvwMeasurementPeriod(16);
        algaeConfig.closedLoop.pid(algaeKp, 0, 0, ClosedLoopSlot.kSlot1);
        coralConfig.closedLoop.pid(coralKp, 0, 0, ClosedLoopSlot.kSlot1);

        algaeMotor.configure(algaeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        coralMotor.configure(coralConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        LaserCan laserCan = LaserCanObjectDetector.createLaserCan(laserCanId, rangingMode);
        ObjectDetector objectDetector = new LaserCanObjectDetector(
                "Coral Detector",
                laserCan,
                maxTrueDistance,
                minTrueDistance);

        IntakeWheelConstants algaeWheelConstants = new IntakeWheelConstants(
                "Intake: Algae",
                algaeKs,
                algaeKv,
                algaeKa,
                algaeWheelRadius,
                reduction,
                MetersPerSecond.of(0.0),
                updatePeriod,
                algaeMaxVelocity,
                algaeMaxAcceleration);

        IntakeWheelConstants coralWheelConstants = new IntakeWheelConstants(
                "Intake: Coral",
                coralKs,
                coralKv,
                coralKa,
                coralWheelRadius,
                reduction,
                MetersPerSecond.of(0.0),
                updatePeriod,
                coralMaxVelocity,
                coralMaxAcceleration);

        IntakeWheel algaeWheel = new NEO550IntakeWheel(
                algaeWheelConstants,
                algaeMotor
        );

        IntakeWheel coralWheel = new NEO550IntakeWheel(
                coralWheelConstants,
                coralMotor
        );

        return new CommandDualIntake(algaeWheel, coralWheel, objectDetector, simLoopPeriod);
    }


}
