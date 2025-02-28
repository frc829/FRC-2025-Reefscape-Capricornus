package frc.robot.subsystems.swerveDrive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Cameras.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.Modules.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.Simulation.*;
import static org.photonvision.PhotonPoseEstimator.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import digilib.swerve.SwerveDriveConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

import digilib.cameras.CameraConstants;
import digilib.cameras.PhotonVisionCamera;
import digilib.swerve.CTRESwerveDrive;
import digilib.swerve.SwerveDriveTelemetry;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;

public class SwerveDriveSubsystemConstants {

    static final class Cameras {

        static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        static final PoseStrategy primaryStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        static final PoseStrategy fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
        static final Matrix<N3, N1> singleTagStdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0, 0.0, 0.0);

        static final class Camera0 {
            static final String name = "Thrifty_Cam_2025";
            static final Distance cameraX = Meter.of(0.0);
            static final Distance cameraY = Meter.of(0.0);
            static final Distance cameraZ = Meter.of(0.0);
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
                    singleTagStdDev);
            static final PhotonCamera photonCamera = new PhotonCamera(name);
            static final PhotonVisionCamera camera = new PhotonVisionCamera(constants, photonCamera);
        }

        static final class Camera1 {
            static final String name = "OV9782-08";
            static final Distance cameraX = Meter.of(0.0);
            static final Distance cameraY = Meter.of(0.0);
            static final Distance cameraZ = Meter.of(0.0);
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
                    singleTagStdDev);
            static final PhotonCamera photonCamera = new PhotonCamera(name);
            static final PhotonVisionCamera camera = new PhotonVisionCamera(constants, photonCamera);
        }

        static final class Camera2 {
            static final String name = "OV9782-07";
            static final Distance cameraX = Centimeters.of(1.0);
            static final Distance cameraY = Centimeters.of(40.5);
            static final Distance cameraZ = Centimeters.of(19.0);
            static final Angle roll = Degrees.of(0.0);
            static final Angle pitch = Degrees.of(0.0);
            static final Angle yaw = Degrees.of(90.0);
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
                    singleTagStdDev);
            static final PhotonCamera photonCamera = new PhotonCamera(name);
            static final PhotonVisionCamera camera = new PhotonVisionCamera(constants, photonCamera);
        }

    }

    static final class Drive {

        static final LinearVelocity maxVelocity = MetersPerSecond.of(4.73);
        static final AngularVelocity maxAngularVelocity = RotationsPerSecond.of(0.75); // 3/4 of a rotation per second max angular velocity
        static final double pathTranslationKp = 10.0;
        static final double pathRotationKp = 5.9918340044856690519902612191937;
        static final SwerveDriveConstants constants = new SwerveDriveConstants(
                maxVelocity,
                maxAngularVelocity,
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
            static final double COUPLE_RATIO = 4.1666666666666666666666666666667;
            static final double DRIVE_GEAR_RATIO = 6.75;
            static final double STEER_GEAR_RATIO = 25;
            static final Distance WHEEL_RADIUS = Inches.of(2);

            static final class Module0 {
                static final Distance xPos = Inches.of(10.25);
                static final Distance yPos = Inches.of(13.5);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.16001);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(3.0715);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.2786);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 82.842;
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
                        static final Voltage ks = Volts.of(0.18619);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.13258);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.027741);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 1.0024E-05;
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
                    static final Angle offset = Rotations.of(-0.020996);
                    static final boolean inverted = false;
                }
            }

            static final class Module1 {
                static final Distance xPos = Inches.of(10.25);
                static final Distance yPos = Inches.of(-13.5);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.080439);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(3.0453);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.43112);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 93.454;
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
                        static final Voltage ks = Volts.of(0.27148);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.12841);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.011464);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 4.5712E-12;
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
                    static final Angle offset = Rotations.of(-0.000977);
                    static final boolean inverted = false;
                }
            }

            static final class Module2 {
                static final Distance xPos = Inches.of(-10.25);
                static final Distance yPos = Inches.of(13.5);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.039418);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(3.03);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.36634);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 89.369;
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
                        static final Voltage ks = Volts.of(0.24392);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.11293);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0049759);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 2.1674E-23;
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
                    static final Angle offset = Rotations.of(-0.381592);
                    static final boolean inverted = false;

                }
            }

            static final class Module3 {
                static final Distance xPos = Inches.of(-10.25);
                static final Distance yPos = Inches.of(-13.5);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.12317);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(3.0633);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.39784);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 90.735;
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
                        static final Voltage ks = Volts.of(0.23051);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.11542);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0092392);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.28655;
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
                    static final Angle offset = Rotations.of(0.049316);
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

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,
     */
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
                        .withSpeedAt12Volts(maxVelocity)
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
                        .withSpeedAt12Volts(maxVelocity)
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
                        .withSpeedAt12Volts(maxVelocity)
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
                        .withSpeedAt12Volts(maxVelocity)
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
                swerveDriveTrain,
                Camera0.camera,
                Camera1.camera,
                Camera2.camera);

        SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(
                CTRE_SWERVE_DRIVE,
                simLoopPeriod);
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.idle());
        return swerveDriveSubsystem;
    }
}
