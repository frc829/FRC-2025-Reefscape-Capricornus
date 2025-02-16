package digilib.swerve;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public interface SwerveDriveRequest {

    public void apply(CTRESwerveDrive CTRESwerveDrive);

    public class FieldCentric implements SwerveDriveRequest {
        private final MutDimensionless maxVelocityPercent = Value.mutable(0.0);
        private final MutDimensionless maxAngularVelocityPercent = Value.mutable(0.0);
        private final MutAngle heading = Radians.mutable(0.0);

        @Override
        public void apply(CTRESwerveDrive CTRESwerveDrive) {
            double vx = Math.cos(heading.baseUnitMagnitude())
                    * CTRESwerveDrive.getMaxVelocity().baseUnitMagnitude()
                    * maxVelocityPercent.baseUnitMagnitude();
            double vy = Math.sin(heading.baseUnitMagnitude())
                    * CTRESwerveDrive.getMaxVelocity().baseUnitMagnitude()
                    * maxVelocityPercent.baseUnitMagnitude();
            double omega = CTRESwerveDrive.getMaxAngularVelocity().baseUnitMagnitude()
                    * maxAngularVelocityPercent.baseUnitMagnitude();
            CTRESwerveDrive.setFieldCentric(vx, vy, omega);
        }

        public FieldCentric withVelocity(Dimensionless maxVelocityPercent) {
            this.maxVelocityPercent.mut_replace(maxVelocityPercent);
            return this;
        }

        public FieldCentric withRotationalVelocity(Dimensionless maxAngularVelocityPercent) {
            this.maxAngularVelocityPercent.mut_replace(maxAngularVelocityPercent);
            return this;
        }

        public FieldCentric withHeadingAngle(Angle headingAngle) {
            this.heading.mut_replace(headingAngle);
            return this;
        }
    }

    public class RobotCentric implements SwerveDriveRequest {
        private final MutDimensionless maxVelocityPercent = Value.mutable(0.0);
        private final MutDimensionless maxAngularVelocityPercent = Value.mutable(0.0);
        private final MutAngle heading = Radians.mutable(0.0);

        @Override
        public void apply(CTRESwerveDrive CTRESwerveDrive) {
            double vx = Math.cos(heading.baseUnitMagnitude())
                    * CTRESwerveDrive.getMaxVelocity().baseUnitMagnitude()
                    * maxVelocityPercent.baseUnitMagnitude();
            double vy = Math.sin(heading.baseUnitMagnitude())
                    * CTRESwerveDrive.getMaxVelocity().baseUnitMagnitude()
                    * maxVelocityPercent.baseUnitMagnitude();
            double omega = CTRESwerveDrive.getMaxAngularVelocity().baseUnitMagnitude()
                    * maxAngularVelocityPercent.baseUnitMagnitude();
            CTRESwerveDrive.setRobotCentric(vx, vy, omega);
        }

        public RobotCentric withVelocity(Dimensionless maxVelocityPercent) {
            this.maxVelocityPercent.mut_replace(maxVelocityPercent);
            return this;
        }

        public RobotCentric withRotationalVelocity(Dimensionless maxAngularVelocityPercent) {
            this.maxAngularVelocityPercent.mut_replace(maxAngularVelocityPercent);
            return this;
        }

        public RobotCentric withHeadingAngle(Angle headingAngle) {
            this.heading.mut_replace(headingAngle);
            return this;
        }
    }

    public class Brake implements SwerveDriveRequest {

        @Override
        public void apply(CTRESwerveDrive CTRESwerveDrive) {
            CTRESwerveDrive.setBrake();
        }
    }

    public class PointWheels implements SwerveDriveRequest {
        private final MutAngle direction = Radians.mutable(0.0);

        @Override
        public void apply(CTRESwerveDrive CTRESwerveDrive) {
            CTRESwerveDrive.setWheels(direction);
        }

        public PointWheels withDirection(Angle direction) {
            this.direction.mut_replace(direction);
            return this;
        }
    }

    public class SeedFieldCentric implements SwerveDriveRequest {
        public void apply(CTRESwerveDrive CTRESwerveDrive) {
            CTRESwerveDrive.setFieldCentricSeed();
        }
    }

    public class ApplyRobotSpeeds implements SwerveDriveRequest {
        private ChassisSpeeds speeds = new ChassisSpeeds();
        private DriveFeedforwards driveFeedforwards = null;

        @Override
        public void apply(CTRESwerveDrive CTRESwerveDrive) {
            CTRESwerveDrive.setApplyRobotSpeeds(speeds, driveFeedforwards);
        }

        public ApplyRobotSpeeds withSpeeds(ChassisSpeeds speeds) {
            this.speeds = speeds;
            return this;
        }

        public ApplyRobotSpeeds withDriveFeedforwards(DriveFeedforwards driveFeedforwards) {
            this.driveFeedforwards = driveFeedforwards;
            return this;
        }
    }

    public class ClockDrive implements SwerveDriveRequest {
        private final MutDimensionless maxVelocityPercent = Value.mutable(0.0);
        private final MutAngle heading = Radians.mutable(0.0);
        private final MutAngle rotation = Radians.mutable(0.0);

        @Override
        public void apply(CTRESwerveDrive CTRESwerveDrive) {
            double vx = Math.cos(heading.baseUnitMagnitude())
                    * CTRESwerveDrive.getMaxVelocity().baseUnitMagnitude()
                    * maxVelocityPercent.baseUnitMagnitude();
            double vy = Math.sin(heading.baseUnitMagnitude())
                    * CTRESwerveDrive.getMaxVelocity().baseUnitMagnitude()
                    * maxVelocityPercent.baseUnitMagnitude();
            CTRESwerveDrive.setClockDrive(vx, vy, rotation.in(Radians));
        }

        public ClockDrive withVelocity(Dimensionless maxVelocityPercent) {
            this.maxVelocityPercent.mut_replace(maxVelocityPercent);
            return this;
        }

        public ClockDrive withRotation(Angle rotation) {
            this.rotation.mut_replace(rotation);
            return this;
        }

        public ClockDrive withHeadingAngle(Angle heading) {
            this.heading.mut_replace(heading);
            return this;
        }
    }

    public class Idle implements SwerveDriveRequest {
        @Override
        public void apply(CTRESwerveDrive CTRESwerveDrive) {
            CTRESwerveDrive.setIdle();
        }
    }






}
