package digilib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutTime;

public class SwerveDriveState {

    private Pose2d pose2d = new Pose2d();
    private final ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private final SwerveModuleState[] swerveModuleStates;
    private final SwerveModuleState[] swerveModuleTargetStates;
    private final SwerveModulePosition[] swerveModulePositions;
    private Rotation2d rawHeading = new Rotation2d();
    private final MutTime timestamp;





}
