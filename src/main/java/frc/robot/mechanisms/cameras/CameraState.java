package frc.robot.mechanisms.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;

import java.util.Optional;

import static edu.wpi.first.units.Units.Seconds;

public class CameraState implements Cloneable {



    public enum CameraMode{
        ROBOT_POSE
    }

    private CameraMode mode;
    private Pose3d robotPose = null;
    private Matrix<N3, N1> robotPoseStdDev = null;
    private final MutTime timestamp = Seconds.mutable(0.0);

    public CameraMode getMode() {
        return mode;
    }

    public Optional<Pose3d> getRobotPose() {
        return Optional.ofNullable(robotPose);
    }

    public Optional<Matrix<N3, N1>> getRobotPoseStdDev() {
        return Optional.ofNullable(robotPoseStdDev);
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public CameraState withCameraMode(CameraMode mode) {
        this.mode = mode;
        return this;
    }

    public CameraState withRobotPose(Pose3d robotPose) {
        this.robotPose = robotPose;
        return this;
    }

    public CameraState withRobotPoseStdDev(Matrix<N3, N1> robotPoseStdDev) {
        this.robotPoseStdDev = robotPoseStdDev;
        return this;
    }

    public CameraState withTimestamp(MutTime timestamp) {
        this.timestamp.mut_replace(timestamp);
        return this;
    }

    public CameraState withCameraState(CameraState cameraState) {
        this.mode = cameraState.getMode();
        this.robotPose = cameraState.getRobotPose().orElse(null);
        this.robotPoseStdDev = cameraState.getRobotPoseStdDev().orElse(null);
        this.timestamp.mut_replace(cameraState.getTimestamp());
        return this;
    }

    @Override
    public CameraState clone() {
        try {
            CameraState toReturn =  (CameraState) super.clone();
            toReturn.mode = mode;
            toReturn.robotPose = robotPose;
            toReturn.robotPoseStdDev = robotPoseStdDev;
            toReturn.timestamp.mut_replace(timestamp);
            return toReturn;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }







}
