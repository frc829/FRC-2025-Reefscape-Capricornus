package frc.robot.subsystems.swerveDrive;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import choreo.Choreo;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import digilib.swerve.SwerveDriveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import digilib.swerve.SwerveDrive;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrive implements Subsystem {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private final SwerveDrive swerveDrive;
    private final Rotation2d blueAlliancePerspectiveRotation;
    private final Rotation2d redAlliancePerspectiveRotation;
    private double lastSimTime;
    private boolean hasAppliedOperatorPerspective = false;
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine sysIdRoutineDrive;
    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine sysIdRoutineSteer;
    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final MutAngularVelocity rotationCharacterizationVelocity = RadiansPerSecond.mutable(0.0);
    private final SysIdRoutine sysIdRoutineRotation;
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final MutLinearVelocity translationCharacterizationVelocity = MetersPerSecond.mutable(0.0);
    private final SysIdRoutine sysIdRoutineTranslation;


    public CommandSwerveDrive(
            SwerveDrive swerveDrive,
            Rotation2d blueAlliancePerspectiveRotation,
            Rotation2d redAlliancePerspectiveRotation) {
        this.swerveDrive = swerveDrive;
        this.blueAlliancePerspectiveRotation = blueAlliancePerspectiveRotation;
        this.redAlliancePerspectiveRotation = redAlliancePerspectiveRotation;

        sysIdRoutineDrive = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,        // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                        null,        // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdDrive_State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        swerveDrive::setDriveCharacterization,
                        null,
                        this
                )
        );

        sysIdRoutineSteer = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,        // Use default ramp rate (1 V/s)
                        Volts.of(7), // Use dynamic voltage of 7 V
                        null,        // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        swerveDrive::setSteerCharacterization,
                        null,
                        this
                )
        );

        sysIdRoutineRotation = new SysIdRoutine(
                new SysIdRoutine.Config(
                        /* This is in radians per second², but SysId only supports "volts per second" */
                        Volts.of(Math.PI / 6).per(Second),
                        /* This is in radians per second, but SysId only supports "volts" */
                        Volts.of(Math.PI),
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        output -> {
                            /* output is actually radians per second, but SysId only supports "volts" */
                            swerveDrive.setRotationCharacterization(rotationCharacterizationVelocity.mut_setMagnitude(output.in(Volts)));
                            /* also log the requested output for SysId */
                            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                        },
                        null,
                        this)
        );

        sysIdRoutineTranslation = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Seconds).of(0.2),        // Use default ramp rate (1 V/s)
                        Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout
                        null,        // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        output -> {
                            swerveDrive.setTranslationCharacterization(translationCharacterizationVelocity
                                    .mut_setMagnitude(output.in(Volts)));
                            SignalLogger.writeDouble("Translational_Rate", output.in(Volts));
                        },
                        null,
                        this
                )
        );
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveDriveRequest> request) {
        return run(() -> swerveDrive.setControl(request.get()));
    }

    public Command applyRequestOnce(Supplier<SwerveDriveRequest> request) {
        return runOnce(() -> swerveDrive.setControl(request.get()));
    }

    public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
        return sysIdRoutineSteer.quasistatic(direction);
    }

    public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
        return sysIdRoutineSteer.dynamic(direction);
    }

    public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
        return sysIdRoutineDrive.quasistatic(direction);
    }

    public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
        return sysIdRoutineDrive.dynamic(direction);
    }

    public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
        return sysIdRoutineRotation.quasistatic(direction);
    }

    public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
        return sysIdRoutineRotation.dynamic(direction);
    }

    public Command sysIdQuasistaticTranslation(SysIdRoutine.Direction direction) {
        return sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command sysIdDynamicTranslation(SysIdRoutine.Direction direction) {
        return sysIdRoutineTranslation.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                swerveDrive.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? redAlliancePerspectiveRotation
                                : blueAlliancePerspectiveRotation
                );
                hasAppliedOperatorPerspective = true;
            });
        }
        swerveDrive.updateTelemetry();
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
            swerveDrive.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    public AutoFactory createAutoFactory() {
        return createAutoFactory((swerveSample, staring) -> {
        });
    }

    public AutoFactory createAutoFactory(Choreo.TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
                swerveDrive::getPose,
                swerveDrive::resetPose,
                swerveDrive::followPath,
                true,
                this,
                trajLogger
        );
    }

    public void configureAutoBuilder() {
        try {
            SwerveDriveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveDriveRequest.ApplyRobotSpeeds();
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    swerveDrive::getPose,   // Supplier of current robot pose
                    swerveDrive::resetPose,         // Consumer for seeding pose against auto
                    swerveDrive::getRobotRelativeSpeeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, driveFeedforwards) -> swerveDrive.setControl(pathApplyRobotSpeeds
                            .withSpeeds(speeds)
                            .withDriveFeedforwards(driveFeedforwards)),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(5.9918340044856690519902612191937, 0, 0)
                    ),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }
}
