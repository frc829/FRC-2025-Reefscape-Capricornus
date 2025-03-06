package digilib.swerve;

import digilib.DigiMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

public class SwerveDriveTelemetry {

    private final StructPublisher<Pose2d> drivePose;
    private final double[] poseArray = new double[3];
    private final DoubleArrayPublisher drivePoseDashboard;
    private final StructPublisher<ChassisSpeeds> driveSpeeds;
    private final double[] speedsArray = new double[3];
    private final DoubleArrayPublisher driveSpeedsDashboard;
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates;
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets;
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions;
    private final DoublePublisher rawHeading;
    private final DoubleArrayPublisher fieldPub;
    private final StringPublisher fieldTypePub;
    private final MechanismLigament2d[] moduleSpeeds;
    private final MechanismLigament2d[] moduleDirections;
    private final double maxVelocityMPS;

    public SwerveDriveTelemetry(
            String name,
            double maxVelocityMPS,
            double maxAngularVelocityRPS) {
        this.maxVelocityMPS = maxVelocityMPS;
        NetworkTable tableData = getDefault().getTable(name + "-Data");
        NetworkTable table = getDefault().getTable(name);
        table.getDoubleTopic("Max Velocity [mps]")
                .publish()
                .set(maxVelocityMPS);
        table.getDoubleTopic("Max Rotational Velocity [dps]")
                .publish()
                .set(maxAngularVelocityRPS * 360.0);
        drivePose = tableData
                .getStructTopic("Pose", Pose2d.struct)
                .publish();
        drivePoseDashboard = table
                .getDoubleArrayTopic("PoseArray")
                .publish();
        driveSpeeds = tableData
                .getStructTopic("Speeds", ChassisSpeeds.struct)
                .publish();
        driveSpeedsDashboard = table
                .getDoubleArrayTopic("SpeedsArray")
                .publish();
        driveModuleStates = tableData
                .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
                .publish();
        driveModuleTargets = tableData
                .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
                .publish();
        driveModulePositions = tableData
                .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
                .publish();
        rawHeading = table
                .getDoubleTopic("Raw Heading [deg]")
                .publish();
        fieldPub = table
                .getDoubleArrayTopic("robotPose")
                .publish();
        fieldTypePub = table
                .getStringTopic(".type")
                .publish();
        Mechanism2d[] moduleMechanisms = new Mechanism2d[]{
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1)};
        moduleSpeeds = new MechanismLigament2d[]{
                moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
                moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
                moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
                moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        };
        moduleDirections = new MechanismLigament2d[]{
                moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        };
        SmartDashboard.putData("Module 0", moduleMechanisms[0]);
        SmartDashboard.putData("Module 1", moduleMechanisms[0]);
        SmartDashboard.putData("Module 2", moduleMechanisms[0]);
        SmartDashboard.putData("Module 3", moduleMechanisms[0]);
    }

    public void telemeterize(SwerveDriveState state) {
        drivePose.set(state.getPose());
        poseArray[0] = DigiMath.roundToDecimal(state.getPose().getX(), 2);
        poseArray[1] = DigiMath.roundToDecimal(state.getPose().getY(), 2);
        poseArray[2] = DigiMath.roundToDecimal(state.getPose().getRotation().getDegrees(), 2);
        drivePoseDashboard.set(poseArray);

        driveSpeeds.set(state.getSpeeds());
        speedsArray[0] = DigiMath.roundToDecimal(state.getSpeeds().vxMetersPerSecond, 2);
        speedsArray[1] = DigiMath.roundToDecimal(state.getSpeeds().vyMetersPerSecond, 2);
        speedsArray[2] = DigiMath.roundToDecimal(state.getSpeeds().omegaRadiansPerSecond * 180 / Math.PI, 2);
        driveSpeedsDashboard.set(speedsArray);

        driveModuleStates.set(state.getModuleStates());
        driveModuleTargets.set(state.getModuleTargets());
        driveModulePositions.set(state.getModulePositions());

        rawHeading.set(state.getRawHeading().getRadians());

        fieldPub.set(poseArray);
        fieldTypePub.set("Field2d");

        for (int i = 0; i < 4; ++i) {
            moduleSpeeds[i].setAngle(state.getModuleStates()[i].angle);
            moduleSpeeds[i].setLength(state.getModuleStates()[i].speedMetersPerSecond / (2 * maxVelocityMPS));
            moduleDirections[i].setAngle(state.getModuleStates()[i].angle);
        }
    }
}
