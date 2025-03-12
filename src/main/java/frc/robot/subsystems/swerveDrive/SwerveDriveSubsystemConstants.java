package frc.robot.subsystems.swerveDrive;

import choreo.auto.AutoFactory;
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
import digilib.cameras.CameraConstants;
import digilib.cameras.PhotonVisionCamera;
import digilib.swerve.CTRESwerveDrive;
import digilib.swerve.SwerveDriveConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Cameras.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.Modules.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.Simulation.simLoopPeriod;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class SwerveDriveSubsystemConstants {

    static final class Cameras {

        static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        static final PoseStrategy primaryStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        static final PoseStrategy fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
        static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                Double.MAX_VALUE);
        static final Matrix<N3, N1> MULTI_TAG_STD_DEVS_AUTO = VecBuilder.fill(0.1, 0.1, 0.1);
        static final Matrix<N3, N1> MULTI_TAG_STD_DEVS_TELEOP = VecBuilder.fill(0.01, 0.01, 0.1);

        static final class Camera0 {
            static final String name = "Front-Camera";
            static final Distance cameraX = Inches.of(10.35);
            static final Distance cameraY = Inches.of(2.6);
            static final Distance cameraZ = Inches.of(20.7);
            static final Angle roll = Degrees.of(0.0);
            static final Angle pitch = Degrees.of(0.0);
            static final Angle yaw = Degrees.of(0.0);
            static final Rotation3d cameraAngle = new Rotation3d(roll, pitch, yaw);
            static final Transform3d robotToCamera = new Transform3d(
                    cameraX,
                    cameraY,
                    cameraZ,
                    cameraAngle);
            static final CameraConstants constants = new CameraConstants(
                    name,
                    robotToCamera,
                    layout,
                    primaryStrategy,
                    fallbackPoseStrategy,
                    640,
                    480,
                    Rotation2d.fromDegrees(70),
                    0.35,
                    0.10,
                    15,
                    50,
                    15,
                    MULTI_TAG_STD_DEVS_TELEOP,
                    MULTI_TAG_STD_DEVS_AUTO,
                    SINGLE_TAG_STD_DEVS);
            static final PhotonCamera photonCamera = new PhotonCamera(name);
            static final PhotonVisionCamera camera = new PhotonVisionCamera(constants, photonCamera);
        }

        static final class Camera1 {
            static final String name = "Back-Camera";
            static final Distance cameraX = Inches.of(-9.2);
            static final Distance cameraY = Inches.of(1.5);
            static final Distance cameraZ = Inches.of(8.5);
            static final Angle roll = Degrees.of(0.0);
            static final Angle pitch = Degrees.of(0.0);
            static final Angle yaw = Degrees.of(180.0);
            static final Rotation3d cameraAngle = new Rotation3d(roll, pitch, yaw);
            static final Transform3d robotToCamera = new Transform3d(
                    cameraX,
                    cameraY,
                    cameraZ,
                    cameraAngle);
            static final CameraConstants constants = new CameraConstants(
                    name,
                    robotToCamera,
                    layout,
                    primaryStrategy,
                    fallbackPoseStrategy,
                    640,
                    480,
                    Rotation2d.fromDegrees(70),
                    0.35,
                    0.10,
                    15,
                    50,
                    15,
                    MULTI_TAG_STD_DEVS_TELEOP,
                    MULTI_TAG_STD_DEVS_AUTO,
                    SINGLE_TAG_STD_DEVS);
            static final PhotonCamera photonCamera = new PhotonCamera(name);
            static final PhotonVisionCamera camera = new PhotonVisionCamera(constants, photonCamera);
        }

    }

    static final class Drive {

        static final double maxVelocityMPS = 4.73;
        static final double maxAngularVelocityRPS = 0.75;
        static final double pathTranslationKp = 10.0;
        static final double deadband = 0.1;
        static final double rotationalDeadband = 0.1;
        static final double pathRotationKp = 8.0984; // 5.9918340044856690519902612191937;
        static final SwerveDriveConstants constants = new SwerveDriveConstants(
                "Swerve Drive",
                maxVelocityMPS,
                maxAngularVelocityRPS,
                deadband,
                rotationalDeadband,
                new PhoenixPIDController(pathTranslationKp, 0, 0),
                new PhoenixPIDController(pathTranslationKp, 0, 0),
                new PhoenixPIDController(pathRotationKp, 0, 0));

        static final class Modules {
            static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
            static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
            static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;
            static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
            static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;
            static final Current SLIP_CURRENT = Amps.of(120.0);
            static final Current steerStatorCurrentLimit = Amps.of(60.0);
            static final boolean steerStatorCurrentLimitEnabled = true;
            static final double COUPLE_RATIO = 4.1666666666666666666666666666667; // 50 tooth first stage / 12 tooth pinion
            static final double DRIVE_GEAR_RATIO = 5.54;  // 16 tooth second stage, 13 tooth pinion
            static final double STEER_GEAR_RATIO = 25;   //12T
            static final Distance WHEEL_RADIUS = Inches.of(2);

            static final class Module0 {
                static final Distance xPos = Inches.of(10.75);
                static final Distance yPos = Inches.of(13.5);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.52503);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(2.4954);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.16477);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 8.0727; //70.0;
                        static final double ki = 0.0;
                        static final double kd = 0.004028; //0.0;
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
                        static final Voltage ks = Volts.of(0.20082);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.13757);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.018818);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.010434;
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
                        static final Voltage ks = Volts.of(0.54212);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(2.4904);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.15858);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 8.0727; //70.0;
                        static final double ki = 0.0;
                        static final double kd = 0.004028; //0.0;
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
                        static final Voltage ks = Volts.of(0.26471);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.12621);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0087653);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.00012557;
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
                        static final Voltage ks = Volts.of(0.51897);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(2.4617);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.1568);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 8.0727; //70.0;
                        static final double ki = 0.0;
                        static final double kd = 0.004028; //0.0;
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
                        static final Voltage ks = Volts.of(0.17571);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.11356);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.01095);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 5.6719E-08;
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
                        static final Voltage ks = Volts.of(0.53513);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(2.4585);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.15239);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 8.0727; //70.0;
                        static final double ki = 0.0;
                        static final double kd = 0.004028; //0.0;
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
                        static final Voltage ks = Volts.of(0.17919);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.11446);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.01078);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 6.948E-08;
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
    private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

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
                layout,
                Camera0.camera,
                Camera1.camera);
        autoFactory = new AutoFactory(
                CTRE_SWERVE_DRIVE.getState()::getPose,
                CTRE_SWERVE_DRIVE::resetPose,
                CTRE_SWERVE_DRIVE::followPath,
                true,
                swerveDriveSubsystem,
                (swerveSample, staring) -> {
                });
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.idle());
        return swerveDriveSubsystem;
    }

    public static AutoFactory getAutoFactory() {
        return autoFactory;
    }
}
