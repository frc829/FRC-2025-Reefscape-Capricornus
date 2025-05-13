package frc.robot.subsystems.intakeWheel;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;
import digilib.intakeWheel.IntakeWheel;
import digilib.intakeWheel.IntakeWheelSubsystem;
import digilib.intakeWheel.TalonFXSIntakeWheel;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static com.ctre.phoenix6.signals.GravityTypeValue.Elevator_Static;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.rio;

public class IntakeSubsystemConstants2 {

    static final String name = "Intake: Coral";
    static final double reduction = 12.0;
    static final double ksVolts = 0.12198;
    static final double kvVoltsPerRPS = 0.12267 * 2 * Math.PI;
    static final double kaVoltsPerRPSSquared = 0.0045787 * 2 * Math.PI;
    static final GravityTypeValue gravityTypeValue = Elevator_Static;
    static final double kpVoltsPerRPS = 0.00011686 * 2 * Math.PI;
    static final double maxControlVoltage = 12.0 - ksVolts;
    static final double maxVelocityRPS = maxControlVoltage / kvVoltsPerRPS;
    static final double maxAccelerationRPSS = maxControlVoltage / kaVoltsPerRPSSquared;

    static final int deviceNumber = 26;
    static final NeutralModeValue neutralModeValue = Brake;
    static final InvertedValue invertedValue = CounterClockwise_Positive;
    static final double supplyCurrentLimitAmps = 70;
    static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(maxAccelerationRPSS);
    static final VoltageConfigs voltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(maxControlVoltage)
            .withPeakReverseVoltage(-maxControlVoltage);
    static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(supplyCurrentLimitAmps)
            .withSupplyCurrentLimitEnable(true);
    static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withInverted(invertedValue)
            .withNeutralMode(neutralModeValue);
    static final Slot1Configs slot1Configs = new Slot1Configs()
            .withKS(ksVolts)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withKV(kvVoltsPerRPS)
            .withKA(kaVoltsPerRPSSquared)
            .withGravityType(gravityTypeValue)
            .withKP(kpVoltsPerRPS);
    static final CommutationConfigs commutationConfigs = new CommutationConfigs()
            .withAdvancedHallSupport(AdvancedHallSupportValue.Enabled)
            .withMotorArrangement(MotorArrangementValue.NEO550_JST);
    static final ExternalFeedbackConfigs externalFeedbackConfigs = new ExternalFeedbackConfigs()
            .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.Commutation)
            .withSensorToMechanismRatio(reduction);
    static final TalonFXSConfiguration config = new TalonFXSConfiguration()
            .withCurrentLimits(currentLimitsConfigs)
            .withVoltage(voltageConfigs)
            .withMotionMagic(motionMagicConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSlot1(slot1Configs)
            .withCommutation(commutationConfigs)
            .withExternalFeedback(externalFeedbackConfigs);
    static final TalonFXS talonFXS = new TalonFXS(deviceNumber, rio);
    static final LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(kvVoltsPerRPS / 2 / Math.PI, kaVoltsPerRPSSquared / 2 / Math.PI);
    static final DCMotorSim sim = new DCMotorSim(plant, DCMotor.getNeo550(1));
    static final Time simLoopPeriod = Seconds.of(0.001);


    public static IntakeWheelSubsystem create() {
        talonFXS.getConfigurator().apply(config);
        MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0).withSlot(1);
        IntakeWheel intakeWheel = new TalonFXSIntakeWheel(
                name,
                maxVelocityRPS,
                maxAccelerationRPSS,
                talonFXS,
                motionMagicVelocityVoltage,
                reduction,
                sim,
                talonFXS.getSimState());
        IntakeWheelSubsystem intakeWheelSubsystem = new IntakeWheelSubsystem(
                intakeWheel,
                simLoopPeriod);
        intakeWheelSubsystem.setDefaultCommand(intakeWheelSubsystem.toVoltage(0.0));
        return intakeWheelSubsystem;
    }
}
