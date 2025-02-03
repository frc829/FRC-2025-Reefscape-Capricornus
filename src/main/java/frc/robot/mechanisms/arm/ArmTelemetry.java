package frc.robot.mechanisms.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class ArmTelemetry {
    // TODO: all fields are private final
    // TODO: create a NetworkTable called armStateTable
    // TODO: create an Angle called minAngle
    // TODO: create an Angle called maxAngle
    // TODO: create an AngularVelocity called maxVelocity
    // TODO: careat an AngularAcceleration called maxAcceleration
    // TODO: create a DoublePublisher called timestamp;
    // TODO: the rest are DoublePublishers
    // TODO: for angle, angularVelocity, minAnglePublisher, maxAnglePublisher, maxVelocityPublisher, maxAccelerationPublisher,
    private final Mechanism2d armMechanism;
    private final MechanismLigament2d armLigament;

    public ArmTelemetry(
            String name,
            Angle minAngle,
            Angle maxAngle,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        // TODO: set the fields for minAngle, maxAngle, maxVelocity, maxAcceleration.  Use the this keyword.
        // TODO: this.minAngle = minAngle, etc.
        // TODO: set this.armStateTable to NetworkTableInstance.getDefault().getTable(name)
        // TODO: set this.timestamp to armStateTable.getDoubleTopic("Timestamp").publish();
        // TODO: set this.angle to armStateTable.getDoubleTopic("Angle").publish()
        // TODO: set this.angularVelocity to armStateTable.getDoubleTopic("AngularVelocity").publish()
        // TODO: repeat for minAnglePublisher, maxAnglePublisher, maxVelocityPublisher, maxAccelerationPublisher
        armMechanism = new Mechanism2d(1, 1);
        armLigament = armMechanism
                .getRoot("ArmRoot", 0.5, 0.5)
                .append(new MechanismLigament2d("Arm", 0.5, 0));
        SmartDashboard.putData("Arm", armMechanism);
    }

    public void telemeterize(ArmState state) {
        // TODO: call minAnglePublisher's set method and pass in minAngle.in(Degrees)
        // TODO: repeat for maxAnglePublisher using maxAngle
        // TODO: repeat for maxVelocityPublisher using maxVelocity
        // TODO: repeat for maxAccelerationPublisher using maxAcceleration
        // TODO: repeat for timestamp using state.getTimeStamp.in(Seconds)
        // TODO: repeat for angle using state.getPosition.in(Degrees)
        // TODO: repeat for angularVelocity using state.getVelocity.in(DegreesPerSecond)
        armLigament.setAngle(state.getPosition().in(Degrees));
    }
}
