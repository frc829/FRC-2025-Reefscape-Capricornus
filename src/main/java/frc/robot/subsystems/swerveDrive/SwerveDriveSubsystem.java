package frc.robot.subsystems.swerveDrive;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;

import java.util.function.Supplier;

import choreo.Choreo.TrajectoryLogger;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import digilib.swerve.SwerveDriveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import digilib.swerve.CTRESwerveDrive;

public class SwerveDriveSubsystem implements Subsystem {
    private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;
    private final CTRESwerveDrive ctreSwerveDrive;
    private double lastSimTime;
    private final Time simLoopPeriod;
    private boolean hasAppliedOperatorPerspective = false;

    public SwerveDriveSubsystem(
            CTRESwerveDrive ctreSwerveDrive,
            Time simLoopPeriod) {
        this.ctreSwerveDrive = ctreSwerveDrive;
        this.simLoopPeriod = simLoopPeriod;

        sysIdRoutinesOnDashboard();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Pose2d getPose() {
        return ctreSwerveDrive.getPose();
    }

    public Command applyRequest(Supplier<SwerveDriveRequest> request) {
        return run(() -> ctreSwerveDrive.setControl(request.get()));
    }

    public Command applyRequestOnce(Supplier<SwerveDriveRequest> request) {
        return runOnce(() -> ctreSwerveDrive.setControl(request.get()));
    }

    Command idle() {
        SwerveDriveRequest.Idle idle = new SwerveDriveRequest.Idle();
        return applyRequest(() -> idle)
                .withName(String.format("%s: IDLE", getName()));
    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                ctreSwerveDrive.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? RED_ALLIANCE_PERSPECTIVE_ROTATION
                                : BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                );
                hasAppliedOperatorPerspective = true;
            });
        }
        ctreSwerveDrive.updateTelemetry();
    }

    public AutoFactory createAutoFactory() {
        return createAutoFactory((swerveSample, staring) -> {
        });
    }

    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
                ctreSwerveDrive::getPose,
                ctreSwerveDrive::resetPose,
                ctreSwerveDrive::followPath,
                true,
                this,
                trajLogger
        );
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            ctreSwerveDrive.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }

    private void sysIdRoutinesOnDashboard() {
        sysIdSteerRoutineOnDashboard();
        sysIdDriveWheelRoutineOnDashboard();
        sysIdRotationRoutineOnDashboard();
        sysIdTranslationRoutineOnDashboard();
    }

    private void sysIdSteerRoutineOnDashboard() {
        Config config = new Config(
                Volts.per(Second).of(1.0),
                Volts.of(7),
                Seconds.of(10),
                state -> SignalLogger.writeString("swerve_steer-sysIdRoutine", state.toString()));
        Mechanism mechanism = new Mechanism(
                ctreSwerveDrive::setSteerCharacterization,
                null,
                this);
        SysIdRoutine routine = new SysIdRoutine(config, mechanism);
        SmartDashboard.putData("Swerve Steer Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Swerve Steer Quasistatic Forward"));
        SmartDashboard.putData("Swerve Steer Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Swerve Steer Quasistatic Reverse"));
        SmartDashboard.putData("Swerve Steer Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Swerve Steer Dynamic Forward"));
        SmartDashboard.putData("Swerve Steer Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Swerve Steer Dynamic Reverse"));
    }

    private void sysIdDriveWheelRoutineOnDashboard() {
        Config config = new Config(
                Volts.per(Second).of(1.0),
                Volts.of(4),
                Seconds.of(10),
                state -> SignalLogger.writeString("swerve_drive_wheel-sysIdRoutine", state.toString()));
        Mechanism mechanism = new Mechanism(
                ctreSwerveDrive::setDriveCharacterization,
                null,
                this);
        SysIdRoutine routine = new SysIdRoutine(config, mechanism);
        SmartDashboard.putData("Swerve Drive Wheel Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Wheel Quasistatic Forward"));
        SmartDashboard.putData("Swerve Drive Wheel Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Wheel Quasistatic Reverse"));
        SmartDashboard.putData("Swerve Drive Wheel Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Wheel Dynamic Forward"));
        SmartDashboard.putData("Swerve Drive Wheel Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Wheel Dynamic Reverse"));
    }

    private void sysIdRotationRoutineOnDashboard() {
        MutAngularVelocity angularVelocity = RadiansPerSecond.mutable(0.0);
        Config config = new Config(
                Volts.of(Math.PI / 6).per(Second), // This is in radians per second²
                Volts.of(Math.PI), // This is in radians per second
                Seconds.of(10),
                state -> SignalLogger.writeString("swerve_rotation-sysIdRoutine", state.toString()));
        Mechanism mechanism = new Mechanism(
                output -> {
                    ctreSwerveDrive.setRotationCharacterization(angularVelocity.mut_setMagnitude(output.in(Volts)));
                    SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                },
                null,
                this);
        SysIdRoutine routine = new SysIdRoutine(config, mechanism);
        SmartDashboard.putData("Swerve Drive Rotation Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Rotation Quasistatic Forward"));
        SmartDashboard.putData("Swerve Drive Rotation Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Rotation Quasistatic Reverse"));
        SmartDashboard.putData("Swerve Drive Rotation Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Rotation Dynamic Forward"));
        SmartDashboard.putData("Swerve Drive Rotation Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Rotation Dynamic Reverse"));
    }

    private void sysIdTranslationRoutineOnDashboard() {
        MutLinearVelocity velocity = MetersPerSecond.mutable(0.0);
        Config config = new Config(
                Volts.per(Seconds).of(0.2), // This is in meters per second²
                Volts.of(1.0), // This is in meters per second
                Seconds.of(10),
                state -> SignalLogger.writeString("swerve_translation-sysIdRoutine", state.toString()));
        Mechanism mechanism = new Mechanism(
                output -> {
                    ctreSwerveDrive.setTranslationCharacterization(velocity.mut_setMagnitude(output.in(Volts)));
                    SignalLogger.writeDouble("Translational_Rate", output.in(Volts));
                },
                null,
                this);
        SysIdRoutine routine = new SysIdRoutine(config, mechanism);
        SmartDashboard.putData("Swerve Drive Translation Quasistatic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Translation Quasistatic Forward"));
        SmartDashboard.putData("Swerve Drive Translation Quasistatic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Translation Quasistatic Reverse"));
        SmartDashboard.putData("Swerve Drive Translation Dynamic Forward",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Translation Dynamic Forward"));
        SmartDashboard.putData("Swerve Drive Translation Dynamic Reverse",
                Commands.runOnce(SignalLogger::start)
                        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop).withName("Swerve Drive Translation Dynamic Reverse"));
    }
}
