package frc.robot.mechanisms.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;

public class WristTelemetry {
    // TODO: all fields are private final
    // TODO: create a NetworkTable called wristStateTable
    // TODO: create an Angle called minAngle
    // TODO: create an Angle called maxAngle
    // TODO: create an AngularVelocity called maxVelocity
    // TODO: create an AngularAcceleration called maxAcceleration
    // TODO: create a DoublePublisher called timestamp;
    // TODO: the rest are DoublePublishers
    // TODO: for angle, angularVelocity, minAnglePublisher, maxAnglePublisher, maxVelocityPublisher, maxAccelerationPublisher,
    private final Mechanism2d wristMechanism;
    private final MechanismLigament2d wristLigamentRight;
    private final MechanismLigament2d wristLigamentLeft;

    public WristTelemetry(
            String name,
            Angle minAngle,
            Angle maxAngle,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        // TODO: set the fields for minAngle, maxAngle, maxVelocity, maxAcceleration.  Use the this keyword.
        // TODO: this.minAngle = minAngle, etc.
        // TODO: set this.wristStateTable to NetworkTableInstance.getDefault().getTable(name)
        // TODO: set this.timestamp to armStateTable.getDoubleTopic("Timestamp").publish();
        // TODO: set this.angle to armStateTable.getDoubleTopic("Angle").publish()
        // TODO: set this.angularVelocity to armStateTable.getDoubleTopic("AngularVelocity").publish()
        // TODO: repeat for minAnglePublisher, maxAnglePublisher, maxVelocityPublisher, maxAccelerationPublisher
        wristMechanism = new Mechanism2d(1, 1);
        wristLigamentRight = wristMechanism
                .getRoot("WristRoot", 0.5, 0.5)
                .append(new MechanismLigament2d("Wrist Ligament Right", 0.5, 0));
        wristLigamentLeft = wristMechanism
                .getRoot("WristRoot", 0.5, 0.5)
                .append(new MechanismLigament2d("Wrist Ligament Left", 0.5, 180));
        SmartDashboard.putData("Wrist", wristMechanism);
    }

    public void telemeterize(WristState state) {
        // TODO: call minAnglePublisher's set method and pass in minAngle.in(Degrees)
        // TODO: repeat for maxAnglePublisher using maxAngle
        // TODO: repeat for maxVelocityPublisher using maxVelocity
        // TODO: repeat for maxAccelerationPublisher using maxAcceleration
        // TODO: repeat for timestamp using state.getTimeStamp.in(Seconds)
        // TODO: repeat for angle using state.getPosition.in(Degrees)
        // TODO: repeat for angularVelocity using state.getVelocity.in(DegreesPerSecond)
        wristLigamentRight.setAngle(state.getPosition().in(Degrees));
        wristLigamentLeft.setAngle(180.0 + state.getPosition().in(Degrees));
    }
}
