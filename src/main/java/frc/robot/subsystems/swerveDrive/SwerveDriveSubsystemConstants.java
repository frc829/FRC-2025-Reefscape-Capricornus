package frc.robot.subsystems.swerveDrive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Cameras.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Drive.Modules.*;
import static frc.robot.subsystems.swerveDrive.SwerveDriveSubsystemConstants.Simulation.*;
import static org.photonvision.PhotonPoseEstimator.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
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
            static final String name = "OV9782-07";
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

    }

    static final class Drive {

        static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(4.73);
        static final double COUPLE_RATIO = 4.1666666666666666666666666666667;
        static final double DRIVE_GEAR_RATIO = 6.75;
        static final double STEER_GEAR_RATIO = 25;
        static final Distance WHEEL_RADIUS = Inches.of(2);
        static final boolean INVERT_LEFT_SIDE = false;
        static final boolean INVERT_RIGHT_SIDE = true;

        static final class Modules {
            static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
            static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
            static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;
            static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
            static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;
            static final Current SLIP_CURRENT = Amps.of(120.0);
            static final Current steerStatorCurrentLimit = Amps.of(60.0);
            static final boolean steerStatorCurrentLimitEnabled = true;


            static final class Module0 {
                static final Distance xPos = Inches.of(10.5);
                static final Distance yPos = Inches.of(13.45);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.09556);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(3.0882);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.18325);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 81.49;
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
                        static final int motorId = 10;
                        static final boolean inverted = false;
                    }
                }

                static final class DriveWheel {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.38569);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.12431);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.010011);
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

                    static final class Motor{
                        private static final int motorId = 20;
                    }

                }

                static final class Encoder{
                    static final int encoderId = 30;
                    static final Angle offset = Rotations.of(0.435546875);
                    static final boolean inverted = false;
                }
            }

            static final class Module1 {
                static final Distance xPos = Inches.of(10.5);
                static final Distance yPos = Inches.of(-13.45);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.093716);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(3.0941);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.21437);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 63.667;
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

                    static final class Motor{
                        static final int motorId = 11;
                        static final boolean inverted = false;
                    }
                }

                static final class DriveWheel {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.22567);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.12133);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.0067662);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.37357;
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

                    static final class Motor{
                        private static final int motorId = 21;
                    }

                }

                static final class Encoder{
                    static final int encoderId = 31;
                    static final Angle offset = Rotations.of(-0.293701171875);
                    static final boolean inverted = false;
                }
            }

            static final class Module2 {
                static final Distance xPos = Inches.of(-10.5);
                static final Distance yPos = Inches.of(13.45);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.032676);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(3.1058);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.20666);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 93.829;
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

                    static final class Motor{
                        static final int motorId = 12;
                        static final boolean inverted = false;
                    }

                }

                static final class DriveWheel {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.19595);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.11845);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.002047);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.19223;
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

                    static final class Motor{
                        private static final int motorId = 22;
                    }

                }

                static final class Encoder{
                    static final int encoderId = 32;
                    static final Angle offset = Rotations.of(0.226318359375);
                    static final boolean inverted = false;

                }
            }

            static final class Module3 {
                static final Distance xPos = Inches.of(-10.5);
                static final Distance yPos = Inches.of(-13.45);

                static final class Steer {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.056407);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(3.1383);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.29554);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseClosedLoopSign;
                        static final double kp = 84.86;
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

                    static final class Motor{
                        static final int motorId = 13;
                        static final boolean inverted = false;
                    }

                }

                static final class DriveWheel {

                    static final class Control {
                        static final Voltage ks = Volts.of(0.2072);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv =
                                Volts.per(RotationsPerSecond).of(0.12913);
                        static final Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka =
                                Volts.per(RotationsPerSecondPerSecond).of(0.010622);
                        static final StaticFeedforwardSignValue staticFeedforwardSignValue =
                                StaticFeedforwardSignValue.UseVelocitySign;
                        static final double kp = 0.35523;
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

                    static final class Motor{
                        static final int motorId = 23;
                    }

                }

                static final class Encoder{
                    static final int encoderId = 33;
                    static final Angle offset = Rotations.of(-0.199951171875);
                    static final boolean inverted = false;
                }
            }
        }

        static final class Gyroscope {

            static final class Control {

            }
        }
    }

    static final class Simulation {
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            // Swerve azimuth does not require much torque output, so we can set a relatively low
                            // stator current limit to help avoid brownouts without impacting performance.
                            .withStatorCurrentLimit(Amps.of(60))
                            .withStatorCurrentLimitEnable(true)
            );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot

    // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
    // This may need to be tuned to your individual robot





    private static final int PIGEON_ID = 40;

    // These are only used for simulation
    private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(Constants.canivore.getName())
            .withPigeon2Id(PIGEON_ID)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorModule0 =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(Drive.STEER_GEAR_RATIO)
                    .withCouplingGearRatio(Drive.COUPLE_RATIO)
                    .withWheelRadius(WHEEL_RADIUS)
                    .withSteerMotorGains(Module0.Steer.Control.slot0Configs)
                    .withDriveMotorGains(Module0.DriveWheel.Control.slot0Configs)
                    .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                    .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                    .withSlipCurrent(SLIP_CURRENT)
                    .withSpeedAt12Volts(Drive.SPEED_AT_12_VOLTS)
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

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorModule1 =
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
                    .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
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

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorModule2 =
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
                    .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
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

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorModule3 =
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
                    .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
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

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreatorModule0.createModuleConstants(
                    Module0.Steer.Motor.motorId,
                    Module0.DriveWheel.Motor.motorId,
                    Module0.Encoder.encoderId,
                    Module0.Encoder.offset,
                    Module0.xPos,
                    Module0.yPos,
                    INVERT_LEFT_SIDE,
                    Module0.Steer.Motor.inverted,
                    Module0.Encoder.inverted
            );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreatorModule1.createModuleConstants(
                    Module1.Steer.Motor.motorId,
                    Module1.DriveWheel.Motor.motorId,
                    Module1.Encoder.encoderId,
                    Module1.Encoder.offset,
                    Module1.xPos,
                    Module1.yPos,
                    INVERT_RIGHT_SIDE,
                    Module1.Steer.Motor.inverted,
                    Module1.Encoder.inverted
            );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreatorModule2.createModuleConstants(
                    Module2.Steer.Motor.motorId,
                    Module2.DriveWheel.Motor.motorId,
                    Module2.Encoder.encoderId,
                    Module2.Encoder.offset,
                    Module2.xPos,
                    Module2.yPos,
                    INVERT_LEFT_SIDE,
                    Module2.Steer.Motor.inverted,
                    Module2.Encoder.inverted
            );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreatorModule3.createModuleConstants(
                    Module3.Steer.Motor.motorId,
                    Module3.DriveWheel.Motor.motorId,
                    Module3.Encoder.encoderId,
                    Module3.Encoder.offset,
                    Module3.xPos,
                    Module3.yPos,
                    INVERT_RIGHT_SIDE,
                    Module3.Steer.Motor.inverted,
                    Module3.Encoder.inverted
            );

    public static final LinearVelocity maxVelocity = SPEED_AT_12_VOLTS; // SPEED_AT_12_VOLTS desired top speed
    public static final AngularVelocity MaxAngularRate = RotationsPerSecond.of(0.75); // 3/4 of a rotation per second max angular velocity

    private static final PhoenixPIDController pathXController = new PhoenixPIDController(10, 0, 0);
    private static final PhoenixPIDController pathYController = new PhoenixPIDController(10, 0, 0);
    public static final PhoenixPIDController pathThetaController = new PhoenixPIDController(5.9918340044856690519902612191937, 0, 0);

    private static final SwerveDriveTelemetry swerveDriveTelemetry = new SwerveDriveTelemetry(maxVelocity);

    private static final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain = new SwerveDrivetrain<>(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);


    private static final CTRESwerveDrive CTRE_SWERVE_DRIVE = new CTRESwerveDrive(
            swerveDriveTrain,
            pathXController,
            pathYController,
            pathThetaController,
            swerveDriveTelemetry,
            maxVelocity,
            MaxAngularRate,
            Camera0.camera,
            Camera1.camera,
            Camera2.camera);

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,
     */
    public static SwerveDriveSubsystem create() {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(
                CTRE_SWERVE_DRIVE,
                simLoopPeriod);
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.idle());
        return swerveDriveSubsystem;
    }
}
