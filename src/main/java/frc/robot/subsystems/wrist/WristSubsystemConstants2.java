package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;
import digilib.wrist.TalonFXSWrist;
import digilib.wrist.Wrist;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import static com.ctre.phoenix6.signals.GravityTypeValue.Elevator_Static;
import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.rio;

public class WristSubsystemConstants2 {

    static final String name = "Wrist";
    static final double reduction = 4.0 * 3.0 * 5.0 * 32.0 / 24.0;   // 10 7 32.0 / 18
    static final double ksVolts = 0.17044;
    static final double kvVoltsPerRPS = 4.9058;
    static final double kaVoltsPerRPSSquared = 0.14377;
    static final GravityTypeValue gravityTypeValue = Elevator_Static;
    static final double positionKpVoltsPerRotation = 6.5235;
    static final double positionKdVoltsPerRPS = 0.0;
    static final double velocityKpVoltsPerRPS = 5.4797E-07;
    static final double maxControlVoltage = 12.0 - ksVolts;
    static final double maxVelocityRPS = maxControlVoltage / kvVoltsPerRPS;
    static final double maxAccelerationRPSS = maxControlVoltage / kaVoltsPerRPSSquared;

    static final int deviceNumber = 17;
    static final NeutralModeValue neutralModeValue = Brake;
    static final InvertedValue invertedValue = Clockwise_Positive;
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
    static final Slot0Configs slot0onfigs = new Slot0Configs()
            .withKS(ksVolts)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withKV(kvVoltsPerRPS)
            .withKA(kaVoltsPerRPSSquared)
            .withGravityType(gravityTypeValue)
            .withKP(positionKpVoltsPerRotation)
            .withKD(positionKdVoltsPerRPS);
    static final Slot1Configs slot1Configs = new Slot1Configs()
            .withKS(ksVolts)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withKV(kvVoltsPerRPS)
            .withKA(kaVoltsPerRPSSquared)
            .withGravityType(gravityTypeValue)
            .withKP(velocityKpVoltsPerRPS);
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
            .withSlot0(slot0onfigs)
            .withSlot1(slot1Configs)
            .withCommutation(commutationConfigs)
            .withExternalFeedback(externalFeedbackConfigs);
    static final TalonFXS talonFXS = new TalonFXS(deviceNumber, rio);
    static final LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(kvVoltsPerRPS / 2 / Math.PI, kaVoltsPerRPSSquared / 2 / Math.PI);
    static final DCMotorSim sim = new DCMotorSim(plant, DCMotor.getNeo550(1));
    static final Time simLoopPeriod = Seconds.of(0.001);
    static final double startingAngleDegrees = 0.0;
    static final double minAngleDegrees = -95.0;
    static final double maxAngleDegrees = 100.0;
    static final MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0).withSlot(0);
    static final MotionMagicVelocityVoltage motionMagicVelocity = new MotionMagicVelocityVoltage(0).withSlot(1);


    public static WristSubsystem create(MechanismLigament2d top, MechanismLigament2d bottom) {
        talonFXS.getConfigurator().apply(config);
        Wrist wrist = new TalonFXSWrist(
                name,
                minAngleDegrees,
                maxAngleDegrees,
                maxVelocityRPS,
                maxAccelerationRPSS,
                reduction,
                startingAngleDegrees,
                talonFXS,
                motionMagicExpoVoltage,
                motionMagicVelocity,
                sim,
                talonFXS.getSimState(),
                top,
                bottom);
        WristSubsystem wristSubsystem = new WristSubsystem(wrist, simLoopPeriod);
        wristSubsystem.setDefaultCommand(wristSubsystem.hold());
        return wristSubsystem;
    }
}
