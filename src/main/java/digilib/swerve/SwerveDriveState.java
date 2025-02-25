package digilib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveState {

    private Pose2d pose = new Pose2d();
    private final ChassisSpeeds speeds = new ChassisSpeeds();
    private final SwerveModuleState[] moduleStates = new SwerveModuleState[]{
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };
    private final SwerveModuleState[] moduleTargets = new SwerveModuleState[]{
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };
    public SwerveModulePosition[] modulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    private Rotation2d rawHeading = new Rotation2d();


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
}
