package frc.robot.subsystems.swerveDrive;

import java.util.function.Supplier;

import choreo.Choreo.TrajectoryLogger;
import com.ctre.phoenix6.Utils;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import digilib.swerve.SwerveDriveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

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
        ctreSwerveDrive.update();
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
}
