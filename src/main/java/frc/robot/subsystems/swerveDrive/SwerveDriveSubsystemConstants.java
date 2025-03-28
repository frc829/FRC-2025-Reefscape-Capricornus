package frc.robot.subsystems.swerveDrive;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import digilib.cameras.Camera;
import digilib.cameras.Limelight3G;
import digilib.swerve.CTRESwerveDrive;
import digilib.swerve.SwerveDriveConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Cameras.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.Modules.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.Simulation.simLoopPeriod;

public class SwerveDriveSubsystemConstants {

    static final class Cameras {

        static final Matrix<N3, N1> robotPoseStdDev = VecBuilder.fill(0.1, 0.1, 999999);

        static final class Camera0 {
            static final String name = "limelight-front";
            static final Distance cameraX = Centimeters.of(24.6);
            static final Distance cameraY = Centimeters.of(-7); // Centimeters.of(6.5);
            static final Distance cameraZ = Centimeters.of(28.5);
            static final Angle roll = Degrees.of(0.0);
            static final Angle pitch = Degrees.of(0.0);
            static final Angle yaw = Degrees.of(0.0);
            static final Rotation3d cameraAngle = new Rotation3d(roll, pitch, yaw);
            static final Transform3d robotToCamera = new Transform3d(
                    cameraX,
                    cameraY,
                    cameraZ,
                    cameraAngle);
            static final Camera.Config config = new Camera.Config(
                    name,
                    robotToCamera,
                    robotPoseStdDev);
            static final Limelight3G camera = new Limelight3G(config);
        }
    }

    static final class Drive {

        static final double maxVelocityMPS = 4.73;
        static final double maxAngularVelocityRPS = 0.75;
        static final double pathTranslationXKp = 3.25;
        static final double pathTranslationYkP = 3.25;
        static final double deadband = 0.0;
        static final double rotationalDeadband = 0.0;
        static final double pathRotationKp = 1.6 * Math.PI; //8.0984; // 5.9918340044856690519902612191937;
        static final SwerveDriveConstants constants = new SwerveDriveConstants(
                "Swerve Drive",
                maxVelocityMPS,
                maxAngularVelocityRPS,
                deadband,
                rotationalDeadband,
                new PhoenixPIDController(pathTranslationXKp, 0, 0),
                new PhoenixPIDController(pathTranslationYkP, 0, 0),
                new PhoenixPIDController(pathRotationKp, 0, 0));

        static final class Modules {
            static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
            static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
            static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;
            static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
            static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;
            static final Current SLIP_CURRENT = Amps.of(80.0);
            static final Current steerStatorCurrentLimit = Amps.of(60.0);
            static final boolean steerStatorCurrentLimitEnabled = true;
            static final double COUPLE_RATIO = 36.0 / 13.0; // 36 tooth first stage / 13 tooth pinion
            static final double DRIVE_GEAR_RATIO = 5.54;  // 16 tooth second stage, 13 tooth pinion
            static final double STEER_GEAR_RATIO = 25;   // 12T
            static final Distance WHEEL_RADIUS = Inches.of(2.0);

            static final class Module0 {
                static final Distance xPos = Inches.of(10.75);
                static final Distance yPos = Inches.of(13.5);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.1);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(2.39);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 100;
                        static final double ki = 0.0;
                        static final double kd = 0.5;
                        static final Slot0Configs slot0Configs = new Slot0Configs()
                                .withKS(ks.baseUnitMagnitude())
                                .withKV(kv.magnitude())
                                .withKA(ka.magnitude())
                                .withStaticFeedforwardSign(staticFeedforwardSignValue)
                                .withKP(kp)
                                .withKI(ki)
                                .withKD(kd);
                    }

                    static final class Motor {
                        static final int id = 10;
                        static final boolean inverted = false;
                    }
                }

                static final class DriveWheel {

                    static final class Control {
                        static final Voltage ks = Volts.of(0);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.124);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.1;
                        static final double ki = 0.0;
                        static final double kd = 0.0;
                        static final Slot0Configs slot0Configs = new Slot0Configs()
                                .withKS(ks.baseUnitMagnitude())
                                .withKV(kv.magnitude())
                                .withKA(ka.magnitude())
                                .withStaticFeedforwardSign(staticFeedforwardSignValue)
                                .withKP(kp)
                                .withKI(ki)
                                .withKD(kd);
                    }

                    static final class Motor {
                        static final int id = 20;
                        static final boolean inverted = false;

                    }

                }

                static final class Encoder {
                    static final int id = 30;
                    static final Angle offset = Rotations.of(-0.032958984375);
                    static final boolean inverted = false;
                }
            }

            static final class Module1 {
                static final Distance xPos = Inches.of(10.75);
                static final Distance yPos = Inches.of(-13.5);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.1);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(2.39);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 100;
                        static final double ki = 0.0;
                        static final double kd = 0.5;
                        static final Slot0Configs slot0Configs = new Slot0Configs()
                                .withKS(ks.baseUnitMagnitude())
                                .withKV(kv.magnitude())
                                .withKA(ka.magnitude())
                                .withStaticFeedforwardSign(staticFeedforwardSignValue)
                                .withKP(kp)
                                .withKI(ki)
                                .withKD(kd);
                    }

                    static final class Motor {
                        static final int id = 11;
                        static final boolean inverted = false;
                    }
                }

                static final class DriveWheel {

                    static final class Control {
                        static final Voltage ks = Volts.of(0);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.124);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.1;
                        static final double ki = 0.0;
                        static final double kd = 0.0;
                        static final Slot0Configs slot0Configs = new Slot0Configs()
                                .withKS(ks.baseUnitMagnitude())
                                .withKV(kv.magnitude())
                                .withKA(ka.magnitude())
                                .withStaticFeedforwardSign(staticFeedforwardSignValue)
                                .withKP(kp)
                                .withKI(ki)
                                .withKD(kd);
                    }

                    static final class Motor {
                        static final int id = 21;
                        static final boolean inverted = true;
                    }

                }

                static final class Encoder {
                    static final int encoderId = 31;
                    static final Angle offset = Rotations.of(-0.00927734375);
                    static final boolean inverted = false;
                }
            }

            static final class Module2 {
                static final Distance xPos = Inches.of(-10.75);
                static final Distance yPos = Inches.of(13.5);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.1);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(2.39);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 100;
                        static final double ki = 0.0;
                        static final double kd = 0.5;
                        static final Slot0Configs slot0Configs = new Slot0Configs()
                                .withKS(ks.baseUnitMagnitude())
                                .withKV(kv.magnitude())
                                .withKA(ka.magnitude())
                                .withStaticFeedforwardSign(staticFeedforwardSignValue)
                                .withKP(kp)
                                .withKI(ki)
                                .withKD(kd);
                    }

                    static final class Motor {
                        static final int id = 12;
                        static final boolean inverted = false;
                    }

                }

                static final class DriveWheel {

                    static final class Control {
                        static final Voltage ks = Volts.of(0);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.124);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.1;
                        static final double ki = 0.0;
                        static final double kd = 0.0;
                        static final Slot0Configs slot0Configs = new Slot0Configs()
                                .withKS(ks.baseUnitMagnitude())
                                .withKV(kv.magnitude())
                                .withKA(ka.magnitude())
                                .withStaticFeedforwardSign(staticFeedforwardSignValue)
                                .withKP(kp)
                                .withKI(ki)
                                .withKD(kd);
                    }

                    static final class Motor {
                        static final int id = 22;
                        static final boolean inverted = false;
                    }

                }

                static final class Encoder {
                    static final int id = 32;
                    static final Angle offset = Rotations.of(-0.3828125);
                    static final boolean inverted = false;

                }
            }

            static final class Module3 {
                static final Distance xPos = Inches.of(-10.75);
                static final Distance yPos = Inches.of(-13.5);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.1);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(2.39);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 100;
                        static final double ki = 0.0;
                        static final double kd = 0.5;
                        static final Slot0Configs slot0Configs = new Slot0Configs()
                                .withKS(ks.baseUnitMagnitude())
                                .withKV(kv.magnitude())
                                .withKA(ka.magnitude())
                                .withStaticFeedforwardSign(staticFeedforwardSignValue)
                                .withKP(kp)
                                .withKI(ki)
                                .withKD(kd);
                    }

                    static final class Motor {
                        static final int id = 13;
                        static final boolean inverted = false;
                    }

                }

                static final class DriveWheel {

                    static final class Control {
                        static final Voltage ks = Volts.of(0);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.124);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.1;
                        static final double ki = 0.0;
                        static final double kd = 0.0;
                        static final Slot0Configs slot0Configs = new Slot0Configs()
                                .withKS(ks.baseUnitMagnitude())
                                .withKV(kv.magnitude())
                                .withKA(ka.magnitude())
                                .withStaticFeedforwardSign(staticFeedforwardSignValue)
                                .withKP(kp)
                                .withKI(ki)
                                .withKD(kd);
                    }

                    static final class Motor {
                        static final int id = 23;
                        static final boolean inverted = true;
                    }

                }

                static final class Encoder {
                    static final int id = 33;
                    static final Angle offset = Rotations.of(0.072509765625);
                    static final boolean inverted = false;
                }
            }
        }

        static final class IMU {
            static final class Gyroscope {
                static final int id = 40;
            }
        }

        static final class Simulation {
            static final Time simLoopPeriod = Seconds.of(0.001);
        }
    }


    // These are only used for simulation
    private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.55496);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.20082);

    private static AutoFactory autoFactory = null;


    public static SwerveDriveSubsystem createCTRESwerveDrive() {
        DeviceConstructor<TalonFX> driveMotorConstructor = TalonFX::new;
        DeviceConstructor<TalonFX> steerMotorConstructor = TalonFX::new;
        DeviceConstructor<CANcoder> encoderConstructor = CANcoder::new;

        TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(steerStatorCurrentLimit)
                        .withStatorCurrentLimitEnable(steerStatorCurrentLimitEnabled));
        CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

        SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(Constants.canivore.getName())
                .withPigeon2Id(IMU.Gyroscope.id)
                .withPigeon2Configs(null);

        SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorModule0 =
                new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                        .withCouplingGearRatio(COUPLE_RATIO)
                        .withWheelRadius(WHEEL_RADIUS)
                        .withSteerMotorGains(Module0.Steer.Control.slot0Configs)
                        .withDriveMotorGains(Module0.DriveWheel.Control.slot0Configs)
                        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                        .withSlipCurrent(SLIP_CURRENT)
                        .withSpeedAt12Volts(maxVelocityMPS)
                        .withDriveMotorType(DRIVE_MOTOR_TYPE)
                        .withSteerMotorType(STEER_MOTOR_TYPE)
                        .withFeedbackSource(STEER_FEEDBACK_TYPE)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs)
                        .withEncoderInitialConfigs(encoderInitialConfigs)
                        .withSteerInertia(STEER_INERTIA)
                        .withDriveInertia(DRIVE_INERTIA)
                        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

        SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorModule1 =
                new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                        .withCouplingGearRatio(COUPLE_RATIO)
                        .withWheelRadius(WHEEL_RADIUS)
                        .withSteerMotorGains(Module1.Steer.Control.slot0Configs)
                        .withDriveMotorGains(Module1.DriveWheel.Control.slot0Configs)
                        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                        .withSlipCurrent(SLIP_CURRENT)
                        .withSpeedAt12Volts(maxVelocityMPS)
                        .withDriveMotorType(DRIVE_MOTOR_TYPE)
                        .withSteerMotorType(STEER_MOTOR_TYPE)
                        .withFeedbackSource(STEER_FEEDBACK_TYPE)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs)
                        .withEncoderInitialConfigs(encoderInitialConfigs)
                        .withSteerInertia(STEER_INERTIA)
                        .withDriveInertia(DRIVE_INERTIA)
                        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

        SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorModule2 =
                new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                        .withCouplingGearRatio(COUPLE_RATIO)
                        .withWheelRadius(WHEEL_RADIUS)
                        .withSteerMotorGains(Module2.Steer.Control.slot0Configs)
                        .withDriveMotorGains(Module2.DriveWheel.Control.slot0Configs)
                        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                        .withSlipCurrent(SLIP_CURRENT)
                        .withSpeedAt12Volts(maxVelocityMPS)
                        .withDriveMotorType(DRIVE_MOTOR_TYPE)
                        .withSteerMotorType(STEER_MOTOR_TYPE)
                        .withFeedbackSource(STEER_FEEDBACK_TYPE)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs)
                        .withEncoderInitialConfigs(encoderInitialConfigs)
                        .withSteerInertia(STEER_INERTIA)
                        .withDriveInertia(DRIVE_INERTIA)
                        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

        final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorModule3 =
                new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                        .withCouplingGearRatio(COUPLE_RATIO)
                        .withWheelRadius(WHEEL_RADIUS)
                        .withSteerMotorGains(Module3.Steer.Control.slot0Configs)
                        .withDriveMotorGains(Module3.DriveWheel.Control.slot0Configs)
                        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                        .withSlipCurrent(SLIP_CURRENT)
                        .withSpeedAt12Volts(maxVelocityMPS)
                        .withDriveMotorType(DRIVE_MOTOR_TYPE)
                        .withSteerMotorType(STEER_MOTOR_TYPE)
                        .withFeedbackSource(STEER_FEEDBACK_TYPE)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs)
                        .withEncoderInitialConfigs(encoderInitialConfigs)
                        .withSteerInertia(STEER_INERTIA)
                        .withDriveInertia(DRIVE_INERTIA)
                        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftConstants =
                ConstantCreatorModule0.createModuleConstants(
                        Module0.Steer.Motor.id,
                        Module0.DriveWheel.Motor.id,
                        Module0.Encoder.id,
                        Module0.Encoder.offset,
                        Module0.xPos,
                        Module0.yPos,
                        Module0.DriveWheel.Motor.inverted,
                        Module0.Steer.Motor.inverted,
                        Module0.Encoder.inverted);
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightConstants =
                ConstantCreatorModule1.createModuleConstants(
                        Module1.Steer.Motor.id,
                        Module1.DriveWheel.Motor.id,
                        Module1.Encoder.encoderId,
                        Module1.Encoder.offset,
                        Module1.xPos,
                        Module1.yPos,
                        Module1.DriveWheel.Motor.inverted,
                        Module1.Steer.Motor.inverted,
                        Module1.Encoder.inverted);
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftConstants =
                ConstantCreatorModule2.createModuleConstants(
                        Module2.Steer.Motor.id,
                        Module2.DriveWheel.Motor.id,
                        Module2.Encoder.id,
                        Module2.Encoder.offset,
                        Module2.xPos,
                        Module2.yPos,
                        Module2.DriveWheel.Motor.inverted,
                        Module2.Steer.Motor.inverted,
                        Module2.Encoder.inverted);
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightConstants =
                ConstantCreatorModule3.createModuleConstants(
                        Module3.Steer.Motor.id,
                        Module3.DriveWheel.Motor.id,
                        Module3.Encoder.id,
                        Module3.Encoder.offset,
                        Module3.xPos,
                        Module3.yPos,
                        Module3.DriveWheel.Motor.inverted,
                        Module3.Steer.Motor.inverted,
                        Module3.Encoder.inverted);

        SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain = new SwerveDrivetrain<>(
                driveMotorConstructor,
                steerMotorConstructor,
                encoderConstructor,
                drivetrainConstants,
                frontLeftConstants,
                frontRightConstants,
                backLeftConstants,
                backRightConstants);


        CTRESwerveDrive CTRE_SWERVE_DRIVE = new CTRESwerveDrive(
                constants,
                swerveDriveTrain);

        SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(
                CTRE_SWERVE_DRIVE,
                simLoopPeriod,
                Camera0.camera);

        TrajectoryLogger<SwerveSample> logger = (trajectory, starting) -> DriverStation.getAlliance().ifPresent(color -> {
            NetworkTable field = getDefault().getTable("Field");
            Pose2d[] poses = color == DriverStation.Alliance.Red
                    ? trajectory.flipped().getPoses()
                    : trajectory.getPoses();
            //noinspection resource
            field.getStructArrayTopic("Trajectory-" + trajectory.name(), Pose2d.struct)
                    .publish()
                    .set(poses);
        });

        autoFactory = new AutoFactory(
                CTRE_SWERVE_DRIVE.getState()::getPose,
                CTRE_SWERVE_DRIVE::resetPose,
                CTRE_SWERVE_DRIVE::followPath,
                true,
                swerveDriveSubsystem,
                logger);
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.idle());
        return swerveDriveSubsystem;
    }

    public static AutoFactory getAutoFactory() {
        return autoFactory;
    }
}
