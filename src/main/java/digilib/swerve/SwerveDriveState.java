package digilib.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.List;

public class SwerveDriveState {

    private Pose2d pose = new Pose2d();
    private final ChassisSpeeds speeds = new ChassisSpeeds();
    private final SwerveModuleState[] moduleStates = new SwerveModuleState[]{
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };
    private final SwerveModuleState[] moduleTargets = new SwerveModuleState[]{
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };
    public final SwerveModulePosition[] modulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    private Rotation2d rawHeading = new Rotation2d();
    private final double[] swerveSteerPositions = new double[modulePositions.length];
    private final double[] swerveSteerVelocities = new double[modulePositions.length];
    private final double[] swerveWheelPositions = new double[modulePositions.length];
    private final double[] swerveWheelVelocities = new double[modulePositions.length];
    private SwerveSample swerveSample = null;


    public Pose2d getPose() {
        return pose;
    }

    public ChassisSpeeds getSpeeds() {
        return speeds;
    }

    public SwerveModuleState[] getModuleStates() {
        return moduleStates;
    }

    public SwerveModuleState[] getModuleTargets() {
        return moduleTargets;
    }

    public SwerveModulePosition[] getModulePositions() {
        return modulePositions;
    }

    public Rotation2d getRawHeading() {
        return rawHeading;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        this.speeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
        this.speeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
        this.speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < moduleStates.length; i++) {
            this.moduleStates[i].speedMetersPerSecond = moduleStates[i].speedMetersPerSecond;
            this.moduleStates[i].angle = moduleStates[i].angle;
        }
    }

    public void setModuleTargets(SwerveModuleState[] moduleTargets) {
        for (int i = 0; i < moduleTargets.length; i++) {
            this.moduleTargets[i].speedMetersPerSecond = moduleTargets[i].speedMetersPerSecond;
            this.moduleTargets[i].angle = moduleTargets[i].angle;
        }
    }

    public void setModulePositions(SwerveModulePosition[] modulePositions) {
        for (int i = 0; i < modulePositions.length; i++) {
            this.modulePositions[i].angle = modulePositions[i].angle;
            this.modulePositions[i].distanceMeters = modulePositions[i].distanceMeters;
        }
    }

    public void setRawHeading(Rotation2d rawHeading) {
        this.rawHeading = rawHeading;
    }

    public double[] getSwerveSteerPositions() {
        return swerveSteerPositions;
    }

    public void setSwerveSteerPositions(List<Double> swerveSteerPositions) {
        for (int i = 0; i < swerveSteerPositions.size(); i++) {
            this.swerveSteerPositions[i] = swerveSteerPositions.get(i);
        }
    }

    public double[] getSwerveSteerVelocities() {
        return swerveSteerVelocities;
    }

    public void setSwerveSteerVelocities(List<Double> swerveSteerVelocities) {
        for (int i = 0; i < swerveSteerVelocities.size(); i++) {
            this.swerveSteerVelocities[i] = swerveSteerVelocities.get(i);
        }
    }

    public double[] getSwerveWheelPositions() {
        return swerveWheelPositions;
    }

    public void setSwerveWheelPositions(List<Double> swerveWheelPositions) {
        for (int i = 0; i < swerveWheelPositions.size(); i++) {
            this.swerveWheelPositions[i] = swerveWheelPositions.get(i);
        }
    }

    public double[] getSwerveWheelVelocities() {
        return swerveWheelVelocities;
    }

    public void setSwerveWheelVelocities(List<Double> swerveWheelVelocities){
        for (int i = 0; i < swerveWheelVelocities.size(); i++) {
            this.swerveWheelVelocities[i] = swerveWheelVelocities.get(i);
        }
    }

    public SwerveSample getSwerveSample() {
        return swerveSample;
    }

    public void setSwerveSample(SwerveSample swerveSample) {
        this.swerveSample = swerveSample;
    }
}
