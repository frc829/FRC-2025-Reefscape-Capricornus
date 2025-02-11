package frc.robot.subsystems.dualIntake;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import digilib.intakeWheel.IntakeWheel;
import digilib.intakeWheel.IntakeWheelConstants;
import digilib.intakeWheel.NEO550IntakeWheel;

import static edu.wpi.first.units.Units.*;

public class CommandDualIntakeConstants {
    private static final Distance algaeWheelRadius = Inches.of(2.0);
    private static final Distance coralWheelRadius = Inches.of(2.0);
    private static final int algaeDeviceNumber = 16;
    private static final int coralDeviceNumber = 26;
    private static final SparkBaseConfig.IdleMode idleMode = SparkBaseConfig.IdleMode.kBrake;
    private static final boolean algaeInverted = false;
    private static final boolean coralInverted = false;
    private static final double reduction = 12.0;
    private static final Voltage algaeKs = Volts.of(0.0);
    private static final Voltage coralKs = Volts.of(0.0);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> algaeKv = Volts.per(RadiansPerSecond).of(8.892526274503728);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> coralKv = Volts.per(RadiansPerSecond).of(8.892526274503728);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> algaeKa = Volts.per(RadiansPerSecondPerSecond).of(0.028028510387446144);
    private static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> coralKa = Volts.per(RadiansPerSecondPerSecond).of(0.028028510387446144);
    private static final double algaeKp = 6.183764634853723E-4;
    private static final double coralKp = 6.183764634853723E-4;
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
    private static final Distance maximumHasElementDistance = Millimeters.of(10.0);

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

        LaserCan laserCan = null;

        try {
            laserCan = new LaserCan(laserCanId);
        } catch (Exception e) {
            System.out.println("Error creating laser can");
        }

        try {
            laserCan.setRangingMode(rangingMode);
        } catch (ConfigurationFailedException e) {
            System.out.println("Error setting laser can ranging mode");
        } catch (NullPointerException e) {
            System.out.println("Laser Can does not exist");
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }


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
                algaeMotor,
                algaeConfig);

        IntakeWheel coralWheel = new NEO550IntakeWheel(
                coralWheelConstants,
                coralMotor,
                coralConfig);

        return new CommandDualIntake(algaeWheel, coralWheel, maximumHasElementDistance, laserCan, simLoopPeriod);
    }


}
